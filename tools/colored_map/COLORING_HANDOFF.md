# 点群着色 品質改善 引き継ぎ (2026-07-18)

> 2026-07-19 追記: residual time offset と6DoF外部較正を dense TUM 軌跡から
> 連続時間で再合成する opt-in stage を追加した。train/held-out の双方で
> LiDAR depth edge と画像edgeの距離が改善した場合だけ
> `transforms_spatiotemporal.json` を採用する。設計とRTK Seq1 smoke結果は
> `docs/research/colored-map-spatiotemporal-calibration-2026-07.md` を参照。
> production版では画像pyramid、移動距離×motion stratified held-out、探索幅を
> 考慮したbound拡張、7軸曲率・条件数・時刻/並進相関の可観測性gateまで追加。
> Seq1 200k点 smokeはtrain 9.20% / held-out 7.85%改善、可観測性PASS。

BIM 枝の `BIM_HANDOFF.md` と同じ趣旨の引き継ぎ文書。README の
camera-coloured flythrough の色が濁っていた問題を起点に、着色パイプライン・
リアルタイムノード・評価系を一気に改善した一連の作業(lidar_slam_ros2
PR #363/#364/#365/#366、全て develop に merge 済み)の現状・API・ハマり所・
再現手順・次の候補をまとめる。

## 1. 何が問題で、何を入れたか

README の RTK-SLAM flythrough (`lidarslam/images/map_flythrough_rtkslam.webp`)
の着色が「白カブリ + 色滲み + 胡椒ノイズ」だった。原因は3つ:

1. **アセットが着色改善前のコードで生成されていた** — 2026-07-12 20:33 生成の
   直後に edge-aware 補間 (#348) / 観測色 medoid (#347) / 露出ゲインクランプ
   (#349) が merge され、README には未反映のままだった。
2. **レンズビネット** — RTK-SLAM cam0 (1600x1200) は四隅が完全に黒く落ちる。
   縁の暗い/黒いピクセルがそのまま点に投影されていた。
3. **Livox `offset_time` 未対応** — `build_lidar_init.py` のデスキュー対象
   フィールド名に無く、mid360 bag は黙って剛体スキャン扱い
   (歩行 ~1 m/s で最大 ~10 cm のスキャン内スミア)。

入れたもの(全て default-off / default 値で既存挙動バイト同一):

| 機能 | 場所 | PR |
|---|---|---|
| `image_margin`(色サンプルのビネット帯除外) | `pointcloud_io.colorize_by_projection_robust` + `--color-image-margin` | #363 |
| `min_samples`(低観測数の色を unseen に降格) | `build_lidar_init._colorize` + `--color-min-samples` | #363 |
| `offset_time` (整数 ns) デスキュー | `build_lidar_init._read_pointcloud_xyz_time` | #363 |
| CPU レンダラ(CUDA/torch 不要) | `render_path.render_frames_cpu` + `--device cpu` | #363 |
| 上記2オプションの realtime 移植 | `scanmatcher/include/scanmatcher/point_colorizer.hpp`(`insideImageMargin` / `PointColor::confirmed`)+ node param `image_margin` / `min_color_samples` | #364 |
| 見た目指標(appearance) | `scripts/evaluate_colored_map_appearance.py` | #365 |
| held-out のビネット除外 truth | `evaluate_heldout_point_colors.py --image-margin` | #365 |
| appearance のゲート組込み | `check_colored_map_quality.py` `appearance_*` 閾値 + pipeline `appearance` ステージ + exp04 プロファイル | #366 |

## 2. 設計上の要点(触る前に知るべき不変条件)

- **margin の意味論**: z バッファ(遮蔽判定)は**フルフレームのまま**。
  margin は「色サンプルの採否」だけを絞る。縁でしか見えない点は
  他ビューの中央から色を得る。offline (`colorize_by_projection_robust`) と
  realtime (`insideImageMargin`) で同一の意味論に揃えてある — 崩さないこと。
- **min_samples は「降格」**: 色を書き換えるのではなく default_rgb (128,128,128)
  に戻して unseen 扱いにする。`render_map_flythrough --color-mode rgb` は
  unseen を drop する、という既存の流れに乗せている。
- **決定論**: これらのオプションは default(0/1)で従来とバイト同一。
  リポジトリの流儀として default-off を守る。
- **この ICP/描画系に GPU は無い**(マシン sasaki-pc は Iris Xe、torch 無し)。
  `render_frames(device='cpu')` が `render_frames_cpu` にディスパッチする。
  仕組み: 量子化深度と点 ID を int64 にパック → `np.minimum.at` 一発で
  ピクセル毎の可視性と勝者 ID を同時解決 → 2x supersample を box 縮小。
  ~0.7 s/frame @600x450, 4.8M 点。gsplat の soft splat と違い不透明ディスク。

## 3. 評価プロトコル(これで測ること)

**3点セット**で評価する。単独指標は騙される:

1. **忠実度**: `evaluate_heldout_point_colors.py --use-pointcloud-colors
   --image-margin <px>`。**非マスク版はビネット画素を正解として数えるため
   margin 付き着色を誤って劣位判定する**(RTK 実測: 非マスク 40.8 vs 42.7 で
   margin なし優位 → マスク版 43.5 vs 40.1 で反転、inlier_20 0.257→0.296)。
2. **見た目**: `evaluate_colored_map_appearance.py` —
   `chroma_retention`(マップ彩度÷画像彩度; washout で <1、モノクロ画像は null)、
   `roughness`(voxel 内色標準偏差 median/p90; 組織化テクスチャは低く
   胡椒ノイズは高い)、`coverage`(点を捨てるチート防止)。
   exp04 で視覚順位を再現済み: 旧着色→再着色→+margin で
   rough_p90 18.5→17.6→12.3、coverage 0.768→0.999。
3. **目視**: `render_map_flythrough.py --test-grid 4 --device cpu` の
   同一視点グリッド比較。

ゲート: `check_colored_map_quality.py --appearance-report ...`。
`configs/colored_map_quality_profiles/hilti_exp04_report_only.yaml` に
report-only の較正済み閾値あり(coverage≥0.95 / rough_med≤2.5 / rough_p90≤15。
旧着色は3つとも violation、現行+margin は余裕 PASS)。

## 4. 採用構成(README アセットの正)

RTK-SLAM construction_seq1、窓 480–545 s(60 m 歩行ループ)、RKO-LIO 軌跡:

```bash
# 1) 軌跡 (CS2 実績の rko_params.ros.yaml を流用、deskew:false / dual_downsample)
timeout -s INT 1800 ros2 run rko_lio offline_node --ros-args \
  --params-file rko_params.ros.yaml -p bag_path:=<construction_seq1> \
  -p imu_topic:=/livox/imu -p lidar_topic:=/livox/points -p base_frame:=base_link \
  -p dump_results:=true -p results_dir:=<out> -p run_name:=seq1_rko
# 2) posed images (time-offset 0 で良い; Kalibr -20.6ms は有意差なしを実測済み)
python3 tools/colored_map/extract_posed_images.py \
  --bag <bag> --traj <tum> --camera-topic /camera/image_raw/compressed \
  --intrinsics-yaml configs/gaussian_splatting/rtk_slam_cam0_intrinsics.yaml \
  --extrinsic configs/gaussian_splatting/rtk_slam_cam0_extrinsic.yaml \
  --undistort --time-offset 0 --start-time 480 --end-time 545 --stride 5 \
  --max-extrapolation 0.2 --out <out>/posed
# 3) 着色点群 (採用パラメータ)
python3 tools/colored_map/build_lidar_init.py \
  --bag <bag> --traj <tum> --points-topic /livox/points \
  --start-time 480 --end-time 545 --voxel 0.015 --min-range 1.5 --max-range 60 \
  --max-points 5000000 --min-neighbors 8 --sparse-voxel 0.1 \
  --color-transforms <out>/posed/transforms.json --color-robust \
  --color-image-margin 140 --color-min-samples 3 --out colored.ply
# 4) レンダ + README アセット
python3 tools/colored_map/render_map_flythrough.py \
  --pointcloud colored.ply --transforms <out>/posed/transforms.json \
  --color-mode rgb --frames 240 --fps 30 --point-size 0.02 --scale 0.375 \
  --loop-fade 12 --device cpu --mp4 master.mp4
ffmpeg -i master.mp4 -vf "fps=15,scale=600:-2:flags=lanczos" -loop 0 \
  -c:v libwebp -quality 78 map_flythrough_rtkslam.webp   # + crf26 mp4 / palette gif
```

## 5. データ・成果物の所在

- **RTK-SLAM bag**: `/media/sasaki/aiueo/datasets/rtk_slam/ros2/construction_seq1`
  (13,180,936,192 bytes)。**注意**: 一度 8.0/13.2 GB で切断破損していた
  ("database disk image is malformed")。`wget -c` (`scripts/download_rtk_slam_dataset.py`
  の URL) で再開修復できる。**使う前にサイズを SEQUENCES の期待値と照合**。
- **今回の RTK 成果物一式**:
  `/media/sasaki/aiueo/benchmarks/rtkslam_seq1_colored_map_20260718/`
  (RKO-LIO TUM 軌跡 `results/seq1_rko_0/`、posed_t0(260 views)、
  colored_{A,B,D}.ply(A=margin なし/B=margin140/D=採用構成)、
  heldout/app/align の各 json、flythrough_master.mp4、新旧比較 PNG)。
  D が README アセットの元。
- **exp04 検証一式**:
  `/media/sasaki/aiueo/datasets/public_validation/hilti_exp04_colored_map_recolor_20260718/`
  (README.md に A/B 表、旧着色との比較 ply、heldout/app json)。
- README アセット: `lidarslam/images/map_flythrough_rtkslam.{webp,mp4,gif}`。
- 経緯の一次資料: `docs/research/3dgs-trajectory-flythrough-notes.md` 追補3、
  `docs/3dgs-map-tutorial.md` の成果物例(再現コマンド)。

## 6. ハマり所(実際に踏んだもの)

- **非マスク held-out を信じない**(§3)。ビネット除外の効果測定には
  `--image-margin` を正解側にも付けるか appearance 指標を使う。
- **colcon の同名パッケージ衝突**: ws に `lidar_slam_ros2` と
  `lidar_slam_ros2_candidate2` が並存し scanmatcher が重複。
  ビルドは **ws ルートから** `source install/setup.bash` 後に
  `colcon build --base-paths lidar_slam_ros2_candidate2 --packages-select scanmatcher`。
  ws の `build/scanmatcher` キャッシュが旧 repo 由来だと
  "does not match the source ... used to generate cache" で落ちる → 消して再生成。
- **offline_node は bag 終端でハング**(仕様)→ `timeout -s INT`。
- **`point_colorizer.hpp` は純ヘッダ** — g++ 単体
  (`g++ -std=c++17 -I scanmatcher/include -I /usr/include/eigen3 ... -lgtest`)
  で colcon なしに即テストできる。
- **posed images のファイル名は `00000.png` 形式**(`frame_XXXXX.png` ではない)。
- **appearance の chroma_retention はモノクロ画像で null** — プロファイルに
  閾値を書くなら色付きリグのみ。
- **exp04 の posed images はグレースケール**(alphasense)。色の議論には
  RTK-SLAM か AIST bag (`/media/sasaki/aiueo/benchmarks/aist_rgb_map/`) を使う。

## 7. 次の候補(未着手、優先度順の私見)

1. **RTK-SLAM 用 colored_map_quality プロファイル**: 色付きリグなので
   `appearance_chroma_retention_min`(~0.9)を初めて有効化できる。
   `benchmarks/rtkslam_seq1_colored_map_20260718/` の json がそのまま較正データ。
2. **ビネットの推定補正**: margin は「捨てる」対策(着色数 -8%)。多数ビューの
   輝度中央値 vs 画像半径から radial gain を推定して「直す」方式なら
   coverage を犠牲にせず縁まで使える。`_sample_pixels` の手前に挟むのが自然。
3. **roughness の平面限定版**: 現行は全 voxel。`project_planar_voxels` で
   平面 voxel に限定すればテクスチャ境界の偽陽性が減り、閾値をさらに絞れる。
4. **CPU レンダラの soft 化**: 不透明ディスクゆえ未マップ領域の黒い隙間が
   gsplat よりも目立つ。supersample 3 化 or 半径依存の soft edge。
5. **リアルタイムノードの実 bag 検証**: #364 は unit test + build まで。
   all-sensors-bag1(`/home/sasaki/autoware_data/all-sensors-bag1`、
   lucid camera_0=camera_top/camera_optical_link、QoS は SensorDataQoS 対応済み)
   で live A/B するとよい。
6. **エクスポート枝への波及**: mesh/LAS/GIS(`mesh_export.py` 等)は改善前の
   色で検証されたまま。D 構成の ply で再検証すると数値が上がるはず。

## 8. 関連 PR / 経緯リンク

- #345/#346–#349: 旧アセット生成と直後の colorizer 改善(すれ違いの原因)
- #363: 着色品質改善 + README アセット再生成(比較画像は PR 参照)
- #364: realtime 移植 + exp04 検証
- #365: appearance 指標 + masked held-out
- #366: appearance のゲート組込み(本文書を含む)

## 9. 追記 (2026-07-18): tools/colored_map/ へ分離

着色は gsplat と無関係、という指摘を受けて本文書を含む着色・エクスポート系
14 モジュールを `tools/gaussian_splatting/` から `tools/colored_map/` へ移動した。
旧パスには **sys.modules リダイレクト式 shim** があり、旧来の
`sys.path.insert(gs_dir)` + `import pointcloud_io` も
`python3 tools/gaussian_splatting/build_lidar_init.py` も動き続ける
(shim は spec 名を先に `sys.modules` へ登録すること — dataclass が
`sys.modules[cls.__module__]` を引くため、忘れると collection で落ちる)。
3DGS 側に残る `train_gsplat` / `render_path` / `render_slam_3dgs_sidebyside`
への依存は、移動側 3 ファイル (build_lidar_init / render_map_flythrough /
colorize_planar_references) の先頭 bootstrap が解決する。
tests / scripts の旧パス参照は shim で無改修動作 — 新規コードは
`tools/colored_map/` を直接参照すること。

## 10. 追記 (2026-07-18): radial vignette correction

未着手候補2を実装した。`colorize_by_projection_robust` の
`vignette_gain_limit` (`build_lidar_init.py` / pipeline では
`--color-vignette-gain-limit`) が複数画像の半径別輝度中央値から共通gain curveを
推定する。内周60%は厳密に1倍、外周だけ単調増加で補正し、指定値でclampする。
default 1.0は補正なしで、従来出力と同一。

RTK-SLAM construction_seq1でDと同じ4,840,318点を使い、marginを探索した。
held-out truthは全て`--image-margin 140`。

| 構成 | margin | coverage | held-out median / inlier20 | rough med / p90 | chroma |
|---|---:|---:|---:|---:|---:|
| D (従来採用) | 140 | 0.7280 | 40.31 / 0.2945 | 5.38 / 20.13 | 1.005 |
| G | 40 | 0.8156 | 43.17 / 0.2650 | 5.00 / 19.04 | 1.002 |
| H | 100 | 0.7658 | 41.75 / 0.2797 | 5.13 / 19.53 | 1.001 |
| **I (採用候補)** | **120** | **0.7476** | **41.11 / 0.2862** | **5.22 / 19.97** | **1.002** |

IはRTK report-only profileの9項目を全てPASSし、D比でcoverageを約2ポイント
回復した。同一4視点gridでも新たな白カブリ・黒ずみ・色滲みは見られなかった。
再現オプションは `--color-image-margin 120
--color-vignette-gain-limit 2.5 --color-min-samples 3`。成果物は同benchmark
directoryの`colored_I_vignette_deadzone_margin120.ply`, `app_I.json`,
`heldout_masked_I.json`, `quality_I.json`, `test_grid_{D,I}.png`。

## 11. 追記 (2026-07-18): planar-only roughness

未着手候補3も実装した。`evaluate_colored_map_appearance.py
--planar-roughness`は既存の全voxel `roughness`を残したまま、PCAで平面と判定した
voxelだけの`planar_roughness`を追加する。default-off。品質プロファイルに
`appearance_planar_roughness_*`閾値があればpipelineが自動で有効化する。

RTK A/B/D/Iの planar median / p90 はそれぞれ 7.20/25.89,
7.68/28.12, 7.50/23.75, 7.16/23.50。RTK profileをmedian<=7.6,
p90<=25.0に較正した。D/IはPASSし、旧A/Bはp90で検出する。平面点率は
約0.72%（約2.6万点、約2千voxel）なので、レポートの`planar_points`と
`voxels_scored`も併せて監視すること。成果物は`app_planar_{A,B,D,I}.json`と
`quality_planar_I.json`。

## 12. 追記 (2026-07-18): realtime実bag A/B

未着手候補5を`/home/sasaki/autoware_data/all-sensors-bag1`で実施した。
LiDARは`/sensing/lidar/concatenated/pointcloud` (`base_link`)、camera_0は
`/lucid_vision/camera_0/{raw_image,camera_info}` (720x465 BGR8)、optical frameは
`camera_top/camera_optical_link`。bagには完成mapがないため、各scanをmap入力として
TF/QoS/投影/confirmationの実データ配線を検証した。

約26秒warm-up後、A (`margin=0,min_samples=1`) は138,760 voxel中22,155 confirmed
(coverage 0.1597, chroma 26.83)、B (`margin=40,min_samples=3`) は156,547中15,075
(coverage 0.0963, chroma 27.32)。Bは低信頼色を期待どおり降格し、TF失敗、node警告、
crashなし。`scripts/evaluate_realtime_colored_map.py`を追加し、成果物は
`realtime_{A,B}.json`。完成world mapでの長時間品質評価は、map topicを含むbagが
得られた時の追加課題。

## 13. 追記 (2026-07-18): CPU soft fill + export再検証

未着手候補4/6を完了した。CPU rendererに`soft_edge_px`、flythrough CLIに
`--soft-edge-px`を追加（default 0で従来とバイト同一）。初版の半透明fringeは
手前点が奥のdiscを覆って暗い輪郭を作ったため不採用。最終版は既存の不透明pixelを
一切変えず、黒いpixelだけを外周fadeで埋める。Iの同一2視点gridでsoft=1は
black pixel率を41.58%から38.43%へ削減、changed fraction 7.50%、平均絶対画素差
2.71。成果物は`test_grid_I_soft{0,1}.png`（失敗版）と
`test_grid_I_soft1_fill_only.png`（最終版）。

I点群のexportも再検証した。GIS CSV / LASを0.1m thinningして851,755点、LASは
全点RGBあり・座標量子1mm。meshには`--thin-voxel`を追加し、0.2m thinning + BPA
(radii 0.15/0.3/0.6m)で127,396 vertices / 109,189 triangles、vertex colourあり。
成果物はbenchmark directoryの`exports_I/`。

## 14. 追記 (2026-07-18): K3着色 + surface cinematic README動画

Iを起点に、画像overlapで共有される可視3D点のRGB比からframe単位の露出・white
balanceを解く`estimate_overlap_rgb_gains`と、voxel PCA normalの入射角・投影scaleで
観測を選ぶview confidenceを追加した。両方default-off。無正則化のKは長いloopで
gain driftが累積し、ほぼ全frameがclampしたため不採用。既存のscalar exposureを
absolute priorにした正則化256と、近距離優先を覆さない弱いangle係数を使うK3を採用。

K3 (`colored_K3_balanced_confidence.ply`) は4,840,318点でcoverage 0.74761
(I: 0.74755)、roughness median/p90 5.43/20.67、planar roughness 7.23/23.89、
chroma retention 1.002。masked held-outはmedian 41.17 / inlier20 0.2863で、RTK
report-only profileの11項目を全PASSした。再着色だけを反復できる
`recolor_pointcloud.py`も追加した。

CPU rendererはvoxel normalを投影してnormal-aligned ellipseを描く
`--surface-splat`を追加。cameraはarc-length pathへcorner slowdown、look-ahead、
近距離構図を加えた`--camera-preset cinematic`を追加した。旧挙動はそれぞれ
default-off / `legacy`。README版は240 frames、600x450、30fps、point size 0.025、
render voxel 0.03、soft edge 1px、surface aspect 2.5、normal voxel 0.12、cinematic。

同じ240 frame評価で旧README→K3はoccupied pixel 0.78820→0.80735、black pixel
0.21180→0.19265、temporal delta p90 0.01589→0.01113、flicker p90
0.00800→0.00688。WebP/MP4/GIFをK3から再生成した。成果物・JSON・棄却したK/K2は
`/media/sasaki/aiueo/benchmarks/rtkslam_seq1_colored_map_20260718/`に保存。

## 15. 追記 (2026-07-19): geometry-aware RGB fusion

K3までのrobust medoid/view confidenceは、同じ3D点に届いた色の選択には強いが、
silhouette隣接pixelへ投影された背景点、深度境界そのもの、移動物体の色を静的地図へ
焼き付ける問題は入力候補の段階で除けなかった。そこでprojection fusionへ次の4 guard
を統合した（全てdefault-off）。

1. 1 pixel z-buffer近傍の最小深度によるforeground silhouette margin
2. 近傍深度rangeによる不連続両側の除外
3. frameごとの外部dynamic maskと可変dilation
4. accepted 7DoF calibration covarianceのrange/focal/motion依存pixel伝播

`attach_dynamic_image_masks.py`はsegmentation modelには依存せず、posed imageと同じ
stemのPNGを検証して`dynamic_mask_path`をframeへ付与する。rootにはmask schema、
全frame完備性、mask pixel率、frame別SHA-256を保存する。pipelineの工程は
`posed images -> dynamic image masks -> calibration -> coloured map`となり、較正済み
transformsにもmask参照と来歴が保持される。動的除外時は部分maskを拒否する。

実装時点の集中回帰は129 pass / 4 skip、graph_based_slam全Python回帰も実行した。
実データ閾値の採用は次段階で行い、coverageだけでなくheld-out色誤差、appearance、
各棄却理由、boundary cropを同時比較する。設計記録は
`docs/research/colored-map-geometry-aware-fusion-2026-07.md`。

## 16. 追記 (2026-07-19): edge-aware sampling性能

`_sample_pixels(..., interp='edge-aware')`の4 corner stack + `np.ptp`を、
bilinear用cornerからin-place min/maxを更新する実装へ置換した。random 1,000座標で
旧式と完全一致。100万座標microbenchmarkはmedian 1.402s→0.519s、RSS
270,092→238,664 KiB。Construction Seq1の同一48.4万点/260画像screenでは
130.53s→97.91s（25.0%短縮）、coverage/report同一、PLY SHA-256も同一だった。
詳細は`docs/research/colored-map-fusion-performance-2026-07.md`。

## 17. 追記 (2026-07-19): release finish

READMEへK3の4.84 M点、coverage 74.76%、11項目profile、動画のoccupied pixel / flicker
改善を明記し、WebPにMP4/GIF fallbackを追加した。公開判断は
`docs/research/colored-map-release-readiness-2026-07.md`へ集約。K3動画を5時点で
目視し、MP4は600x450 / 30 fps / 240 frames / 8秒、GIFは480x360 / 10 fps /
80 frames / 8秒を確認した。geometry boundary guardを既定化せず、既存planar gateを
維持する制約もREADME、release note、release recordで一致させた。

## 18. 追記 (2026-07-19): K4 pose-aware dynamic cleaning

README点群を実際に更新するため、`dynamic-object-removal` 0.5.0のoffline
`fusion`を`build_lidar_init.py`へ任意依存・default-offで統合した。deskew済みscanを
K3と同じRKO-LIO軌跡でworldへ移し、同じposeのsensor originと組にして判定する。
完成地図だけを後処理する構成ではない。pipelineはalgorithm/version、evidence scan、
閾値、除去数を`dynamic_map_cleaning.json`へ保存する。

Construction Seq1の639 scanでevery-5thの128 scanを証拠に使い、pre-cap
6,193,579点中884,580点（14.28%）を除外した。default-off再構築は既存K3の
4,840,318 XYZと完全一致。K4は4,906,133点、coverage 0.7666、held-out median /
inlier20は40.54 / 0.2909、global roughness 5.20/19.98、planar roughness
6.40/23.52、chroma retention 1.0051で11項目を全PASSした。K3→K4でcoverage、
held-out、global/planar roughnessがすべて改善したためREADME動画へ昇格した。

このsequenceにpoint-wise動的GTはない。14.28%を「動物体の正解率」と解釈せず、
same-pose provenance、K3 exact rebuild、双方向structure support、品質profile、同一8視点
目視を採用根拠とする。詳細は
`docs/research/colored-map-dynamic-cleaning-2026-07.md`。

## 19. 追記 (2026-07-19): K5 residual diagnostics

K4の11項目PASSが目視品質を代表していなかったため、camera-LiDAR alignmentへ
signed x/y残差、対応率、方向coherence、ワーストview overlay、contact sheetを追加した。
pipelineでは`--alignment-diagnostics`で品質評価と同時生成できる。通常動作は不変。

K4の4,906,133点・26 view実測では、ワースト10 viewの12 px圏外率が
36.5%〜54.2%、全体の方向coherenceは0.032だった。一定方向の外部較正ずれだけでは
説明できず、棚・天井・物体境界にscene-dependentな未対応edgeが広がる。成果物は
`benchmarks/rtkslam_seq1_colored_map_20260718/k5_diagnostics/`。次はunsupportedな
LiDAR depth edgeを較正目的から除外した後、時刻・motion distortion・軌跡依存残差を
別々に評価する。

surface-supported edge gateもdefault-offで実装し、raw edge数と保持率、較正時の
最小保持率gate（pipeline既定25%）を追加した。しかしK4全点のr2/n4は51.42%を保持して
median 7.759→7.234 px、圏外率34.96→32.79%に留まり、p90は13 pxのまま。productionの
30万点subsampleでは保持率7.62%まで崩れたため不採用。見かけのmedian 4.59 pxは
selection biasであり、保持率gateが正しく拒否する。次は画像平面の点密度に依存しない
correspondence supportが必要。

続いて全密度4.91M点の初期z-buffer winnerからview別3D contour point IDを固定し、
30万点subsampleや候補姿勢で評価集合が変わらない`--fixed-contours`を追加した。
画像edgeから12 px以内を先に選ぶ方式は初期姿勢を最適値へlockし、train/held-outとも
改善0%だったためoptimizationでは明示拒否する。geometry-only（association=0）の
13 view×20k点smokeは260k点を100%保持し、train 2.25%改善に対しheld-outは
0.76%（7.3723→7.3160 px）のみ。stationary pointもlocal範囲外で可観測性FAILとなり
不採用。密度非依存の証拠集合は完成したが、clutter環境のnearest image-edge目的が
まだ曖昧で、K4のpose/README assetは更新していない。

固定contourへdepth discontinuity normalを持たせ、画像gradient normalとの角度も見る
`--orientation-max-angle-deg`をdefault-offで追加した。30度の13 view×20k smokeは
全260k点を母集団に残したまま未対応61.24%、train改善1.09%、held-out改善0.52%。
search boundaryへ到達し、曲率不足/不安定、time-translation pair非可観測、local外の
stationary point、ill-conditionでもFAILしたため不採用。疎depth rasterのpixel normalは
棚・角・細構造で不安定。次は角度を緩めるのでなく、contourを支持付きline segmentへ
束ねてsegment tangentをrobust推定する。K4 pose/assetは引き続き未変更。
