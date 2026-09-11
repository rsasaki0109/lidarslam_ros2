# LiDAR 退化耐性 A/B: radar Doppler 融合と intensity 拘束 (2026-07-16)

NTNU LiDAR Degeneracy Datasets (fog 廊下 / tunnel、[paper](https://ar5iv.org/abs/2403.05332)、
[dataset](https://github.com/ntnu-arl/lidar_degeneracy_datasets)) に対する
RKO-LIO の退化対策 A/B 評価の記録。方針は「弱い方向だけを外部センサで補う」
([参考スライド](https://speakerdeck.com/naokiakai/lidar-slamnoshi-zhuang-tosensarong-he-liequn-karacontinuous-time-liomade?slide=25))。
GT が無いため指標は始点–終点ずれ (fog はループ収録、真値 0 m) と
到達距離 (tunnel は片道 ~500 m)。`scripts/evaluate_degeneracy_trajectory.py` で算出。

## 結果サマリ

| 系列 | 構成 | 始点–終点ずれ / 到達 | path | 備考 |
|---|---|---|---|---|
| fog | baseline | 32.798 m | 177.4 m | |
| fog | IMU 弱方向 prior | 32.817 m | 177.3 m | 介入 84 scan、改善なし |
| fog | radar Hessian 弱方向 prior | 32.864 m | 177.3 m | 融合 84 scan、改善なし |
| fog | radar 速度不一致ゲート w=0.25 | 30.976 m | 148.0 m | 補正 849 scan |
| fog | radar 速度不一致ゲート w=0.7 | 26.835 m | 156.5 m | |
| **fog** | **radar 速度不一致ゲート w=1.0** | **22.684 m (−31%)** | 164.2 m | 採用構成 |
| tunnel | baseline | 到達 98.67 m | 233.8 m | 真値 ~500 m の 20% |
| **tunnel** | **radar 両ゲート併用** | **到達 457.05 m (−8.6%)** | 477.4 m | Hessian prior 1981 + 不一致補正 1011 scan |
| tunnel | intensity プロファイル拘束 | 到達 98.69 m | 233.8 m | 適用 278 scan、改善なし (下記) |

公式 run: `/media/sasaki/aiueo/benchmarks/lidar_degeneracy_datasets_v1/runs/`
(`fog_rko_lio_radar_disagreement_v1`, `tunnel_rko_lio_radar_disagreement_v1`,
`tunnel_rko_lio_intensity_v1` ほか)。radar 入り rosbag2 は同 `ros2_radar/{fog,tunnel}`
(`normalize_lidar_degeneracy_rosbag.py --radar-input` で生成、LiDAR/IMU は既存 bag と
メッセージ数完全一致)。

## 知見 1: fog の退化は「情報不足」ではなく「クラッタロック」

- fog 区間 (0–110s) はリターンが 3.7k–8.8k 点まで減り、エアロゾル点群が
  センサと共に動くため ICP は健全な Hessian で「ほぼ静止」を出力する。
- 固有値ベースの弱方向ゲートは 3e-4 まで緩めても fog 区間で一度も発火しない。
  発火するのは録画末尾の晴れた廊下 (110–172s) だけで、そこでは radar と LIO が
  一致しており補正の余地がない。
- 証拠: IMU 歩行振動 (acc std ~0.5 m/s²) は 10s 以降常時存在、radar body-x 変位
  積分 96.9 m vs LIO 64.2 m。欠損 ≈ endpoint 誤差 32.8 m と整合。

対策として `radar_disagreement_gate` を実装: radar ego-velocity と ICP 速度の
不一致が `radar_disagreement_min_mps` (0.2) を `radar_disagreement_min_scans` (10)
連続で超えたら、radar 観測方向**のみ**に沿って並進を `radar_disagreement_weight`
で radar 変位へブレンド。クラッタロック中の ICP は「弱い」のではなく「能動的に
誤る」ため全置換 (w=1.0) が最良 (重み単調: 0.25→31.0 m, 0.7→26.8 m, 1.0→22.7 m)。

## 知見 2: tunnel は Hessian ゲートと不一致ゲートが相補的に効く

tunnel では固有値ゲートが正しく発火し (1981 scan で radar prior 融合)、加えて
不一致ゲートが 1011 scan を補正。到達 98.7 → 457.1 m (真値の 91%)、横 RMS 1.93 m。
残り −8.6% は radar 速度の系統的過小 (狭 FOV single-chip の観測限界、fog 検証で
speed 比 ~0.62、前方軸はより良い) と整合。

追補: `radar_velocity_scale` (node param、default 1.0) を追加し tunnel で
スイープした結果、**1.05 で到達 495.3 m (誤差 −0.9%)**、1.10 は 519.8 m
(+4.0%) で行き過ぎ。fog は 1.05 でわずかに悪化 (22.68 → 24.34 m) — fog の
補正方向の radar 速度はほぼ不偏のため。スケールはジオメトリ依存の較正値
として tunnel 専用 param
(`rko_lio_lidar_degeneracy_radar_weak_direction_tunnel.{yaml,ros.yaml}`) に
1.05 を置き、共有デフォルトは 1.0 を維持 (真値 ~500 m への tuned 値である
ことに注意)。run: `runs/tunnel_rko_lio_radar_scale105_v1`。

## 知見 3: intensity プロファイル拘束は「ゲートのカバレッジ」で死ぬ (負結果)

反射率 1D プロファイルのスキャン間相関 (`intensity_profile.hpp`、相関 0.85–0.98)
で弱方向変位を測り既存 prior スロットに注入する実装は設計どおり動くが、
tunnel の endpoint は不変。原因は fog と同型:

- 確定弱方向ウィンドウは全行程の ~17% (末尾 ~55s) のみ。距離欠損 2.3 倍は
  残り 83% の「ゲートが確定しない軟らかい退化」で蓄積する。
- ゲート内では IMU/幾何初期推定が既に良く、測定シフトはノイズ中心
  (mean −0.001 m, std 0.027 m)。閾値緩和 (相関 0.5 / 25 bin) でも不変を確認。

教訓: **退化補正の効果はゲートのカバレッジで決まる**。Hessian 固有値は
「硬い退化」しか拾えず、蓄積誤差の主因である「軟らかい退化 / クラッタロック」は
センサ間不一致 (radar vs ICP) のような一貫性シグナルでしか捕まらない。
[BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO) (RSS 2026) は同じ問題を
voxel 毎 oriented height image への直接レジストレーションで解いており、
テクスチャ登録を「常時オン」にする設計はこの教訓と整合する。

## 知見 4: intensity 不一致ゲート化で radar なしでも tunnel を部分回復

知見 3 の教訓どおり、intensity 由来速度 vs ICP 速度の不一致ゲート
(`intensity_disagreement_gate`、radar 版と同型、軸は運動方向で Hessian 不要) に
載せ替えた結果:

- **tunnel (radar なし)**: 到達 98.67 → **153.75 m**、path 233.8 → **413.8 m**
  (補正 635 scan)。radar 併用の 457 m には届かないが、radar 非搭載リグで
  実質的な回復。
- **fog**: 36.97 m と**ベースラインより悪化**。エアロゾルは相関こそ出る
  (706/744) が変位が誤誘導する。fog は radar 必須、intensity は自己相似幾何+
  明瞭な反射テクスチャ (tunnel) 向け、という物理的に妥当な住み分け。
- 実装上の実バグを1件発見・修正: 相関が取れないスキャンで streak をリセット
  していた (「測定なし」と「一致」の混同)。リセットは「測定された一致」か
  静止時のみに限定。valid-shift 率 ~40% に合わせ min_scans 10→3。

run: `runs/tunnel_rko_lio_intensity_disagreement_v1`,
`runs/fog_rko_lio_intensity_disagreement_v1`。param:
`rko_lio_lidar_degeneracy_intensity_disagreement.{yaml,ros.yaml}` (default off)。

### 追試 (2026-07-17): HILTI exp07 では全閾値で悪化 — 適用条件の確定

mm-GT のある HILTI 2022 exp07 (長い自己相似廊下、baseline APE 0.318 m) に
適用した結果、min_mps 0.2/0.05/0.02 のいずれでも **APE 0.94〜2.39 m と
2.9〜7.5 倍悪化**。原因は閾値ではなく**相関のエイリアシング**: 自己相似廊下では
反射テクスチャ自体も走行軸に沿って周期的で、相関 ≥0.6 の「もっともらしく
誤った」シフトが定常的に発生し (52% の試行が閾値超過)、weight 1.0 で直接
注入されて累積する。fog (テクスチャがセンサ随伴ノイズ) と鏡像の失敗モード。

**適用条件の結論**: intensity 不一致ゲートが有効なのは「幾何は自己相似だが
反射テクスチャは特徴的 (照明・標識・ケーブル等)」な環境のみ (NTNU tunnel が
該当)。幾何もテクスチャも自己相似な環境 (HILTI exp07) とテクスチャが偽物の
環境 (fog) では有害。default-off の環境別オプトインが正しい運用。対策候補
(未実装): min_correlation 引き上げ、profile 長短縮、weight < 1 の減衰、
シフト分布の多峰性検出によるエイリアス棄却。
benchmark: `/media/sasaki/aiueo/benchmarks/hilti_exp07_intensity_disagreement_20260717/`。

## 実装 (Thirdparty/rko_lio、全て default-off で既存挙動バイト同一)

- `core/radar_ego_velocity.hpp`: /radar/cloud (x,y,z,intensity,velocity) から
  RANSAC LSQ で ego 速度。符号 v_r = −(d·v)。radar→IMU extrinsic は公式値
  quat_xyzw [0.953717, 0, −0.3007058, 0] (Wahba 独立推定と 7.75° 一致)。
- `core/lio.{hpp,cpp}`: `RadarVelocityPrior` (one-shot) + Hessian 弱方向 prior
  差し替え + `radar_disagreement_gate` (post-ICP、radar 方向限定並進補正)。
- `core/intensity_profile.hpp`: プロファイル構築 + 正規化相互相関シフト推定
  (放物線サブビン補間)。
- `ros/node.cpp`: radar 購読/オフライン bag ディスパッチ、**時刻付き prior
  キュー** (offline は reader が実処理より ~9s 先行するため latest-wins では
  全滅する)、summary json ダンプ。
- param: `lidarslam/param/rko_lio_lidar_degeneracy_radar_weak_direction.{yaml,ros.yaml}`,
  `rko_lio_lidar_degeneracy_intensity.{yaml,ros.yaml}`,
  `lidarslam_lidar_degeneracy.yaml` (backend パススルー設定)。
- テスト: `test_radar_ego_velocity` (5), `test_intensity_profile` (6),
  既存 `test_degeneracy_aware_solve` (13) 等すべて green。

## 運用メモ

- offline_node は bag 終端で必ずハング (最終スキャンの sweep 終端 > 最終 IMU
  時刻で consumer が永久待ち)。`timeout -s INT` で止めれば dump される。
- graph_based_slam をデフォルト param で並走させると event-driven ループ探索が
  詰まり DDS バックプレッシャで全体が停止する。退化ベンチは
  `lidarslam_lidar_degeneracy.yaml` (loop stride 1000000) を使う。

## 知見 5 (2026-07-17): 連続情報重み付き radar 融合で fog 26.17 → 11.21 m

不一致ゲートの構造的取り逃し (起動遅れ・閾値未満・radar 方向外) を解消する
`radar_velocity_continuous_fusion` を実装: 毎スキャン post-ICP で ICP 速度と
radar 速度を情報重み付きベイズ融合 (radar 側はセンサ軸別 σ: 前方 0.06 /
横・縦 0.5 m/s、ICP 側は固定トラストスケール 0.2)。

- fog: 26.17 → **11.21 m** (baseline 35.57 m、新アーキ)。tunnel: 505.0 m 維持。
  決定論バイト一致、テスト全 green。
- **実装上の発見**: 本実装の ICP Hessian 並進ブロックは対応点数正規化 +
  並進ヤコビアン=単位行列のため**常に ≈I** で、幾何的信頼度を運ばない
  (クラッタロックの「自信満々の誤り」と本物の確信を区別できない)。
  情報重みの ICP 側は固定スケールとするしかなく、これが loose 結合の限界。
- DR-LRIO (密結合) の <1 m には未達。残差は yaw ドリフト蓄積 + radar バイアス
  で、逐次補正でなく同時最適化 (密結合) が必要という結論。
- config: `rko_lio_lidar_degeneracy_radar_continuous.{yaml,ros.yaml}`、
  プリセット `presets/corridor_fog_radar.ros.yaml` にも反映済み。
