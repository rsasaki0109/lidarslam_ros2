# RTK-SLAM total-station GT — methodology for the mid360_gt_rtkslam_* profiles (v0.5)

`mid360_vs_glim`（GLIM 推定との cross-validation）を実 GT に置換した v0.5 の
リリースゲート方法論。データセット・メトリクス定義・帰属表記をここに固定する。
経緯と検証ログは `docs/roadmap/v0.5.md` §3。

## データセット

**RTK-SLAM Dataset** — Zhang, Ress, Skuddis, Soergel, Haala (Univ. Stuttgart),
arXiv:2604.07151, **CC-BY 4.0**。
bags: huggingface.co/datasets/Willyzw/rtk-slam-dataset（ROS 2, 計 182 GB）/
GT + 例軌跡: github.com/Willyzw/rtk-slam-eval（数 MB の git clone で取得可能、
`scripts/download_rtk_slam_dataset.py --eval-assets`）。

- センサ: Livox MID-360（`/livox/points` = PointCloud2、`/livox/lidar` は
  CustomMsg なので使わない）+ `/livox/imu` 200 Hz。
- GT: **geodetic total station が測量した疎な checkpoint**
  （CSV `point_id,easting,northing,height,env,timestamp`、16–36 点/シーケンス）。
  オンボード RTK から独立した測量器による third-party GT。
- 4 シーケンス: Construction Hall 1/2（屋内、16 chkpt each）、
  Stadtgarten 1/2（屋外公園、36/19 chkpt）。

## メトリクス: SE(3)-aligned checkpoint RMSE

- `scripts/generate_rtk_slam_reference.py` が checkpoint CSV を最古点原点の
  sparse TUM に変換（identity quat）。timestamp は bag と同一 Unix-epoch
  センサクロック（実測確認済み）。
- `scripts/write_aligned_trajectory_metrics.py` が推定軌跡と最近傍時刻マッチ →
  **Umeyama SE(3) アライン後の RMSE**（`evo.ape.rmse`、既存 `ape_rmse_gt_m`
  と同一定義）。**`--match-tolerance 2.0` 必須**（dataset 公式の max_dt。
  デフォルトの 0.05→0.15 s カスケードでは疎 checkpoint を silent drop する）。
- dataset 公式の zero-aligned 絶対 RMSE / gap% は GNSS アンカーが前提
  （RKO-LIO は LiDAR-inertial のみ）なので**ゲートには使わない**。SE(3)
  アラインは `livox_frame → base` の定数 extrinsic も吸収する（identity で可）。
- **dense トラジェクトリ（RKO-LIO raw odometry）を採点する**。loop-closed
  `/modified_path` は submap ノード単位で疎になり、checkpoint（測量滞在 =
  低速 = submap が立たない）がギャップに落ちて採点不能。公開ベースラインも
  dense 形式で採点されており土俵が同じ。

## 再現用 accuracy suite

シーケンスごとの RKO-LIO preset、疎 checkpoint の association、bag 終端判定を
手入力せず、リポジトリ管理の contract から実行する。

```bash
python3 scripts/download_rtk_slam_dataset.py --sequence construction_seq2 \
  --eval-assets
python3 scripts/run_rtk_slam_accuracy_suite.py --dry-run
python3 scripts/run_rtk_slam_accuracy_suite.py
```

既定は最小の `construction_seq2`。複数は `--sequence` を繰り返し、全4本は
`--sequence all` を使う。大容量データを別ディスクに置く場合だけ
`--dataset-root <rtk_slam>` を指定する。exact command は `suite_plan.json`、
最終結果は `suite_summary.json` および各シーケンスの `metrics.json` に残る。

## 結果(自前 RKO-LIO、v0.5 時点)

| シーケンス | 環境 | config | chkpt | RMSE (m) | median | 公開手法(参考) |
|---|---|---|---|---|---|---|
| construction_seq2 | 屋内 | indoor | 16/16 | **0.154** | 0.061 | fast_lio_sam 0.086 / okvis 0.075 |
| construction_seq1 | 屋内(最難) | indoor | 16/16 | **0.403** | 0.263 | 0.221 / 0.227 |
| stadtgarten_seq2 | 屋外公園 | outdoor | 19/19 | **0.835** | 0.327 | 0.070 / 0.083 |
| stadtgarten_seq1 | 屋外公園(26分/1km) | outdoor | 36/36 | **1.666** | 1.511 | 0.071 / 0.054 |

indoor config = `configs/mid360_robot/rko_lio_rtk_slam_mid360.yaml`
（voxel 0.5, double_downsample on）。

## outdoor config: double_downsample が律速だった

indoor config のまま屋外公園を走らせると RMSE 3.903 m（median 1.37, max 11.7）
まで漂流する。スイープの結果、**`double_downsample: false` の 1 点だけで
0.835 m（4.7 倍改善）**。疎・遠方特徴の公園では間引き 2 段目が correspondence
を飢餓させていた。voxel 1.0 + corr 1.0（rko_lio 屋外既定スケール）は 2.348 m で
むしろ悪く、voxel 0.5 を維持して間引きだけ止めるのが正解だった。
outdoor config = `configs/mid360_robot/rko_lio_rtk_slam_mid360_outdoor.yaml`。

| stadtgarten_seq2 config | RMSE (m) | median | max |
|---|---|---|---|
| indoor（voxel 0.5, DD on） | 3.903 | 1.366 | 11.75 |
| **voxel 0.5, DD off（採用）** | **0.835** | **0.327** | **3.05** |
| voxel 1.0, corr 1.0, DD off | 2.348 | 0.468 | 8.96 |

## v0.9 RKO-LIO voxel backend再検証

RKO-LIO `d0923f4` では、v0.5時点のBonxai mapとunordered-map voxel samplerが
robin-map + input-order samplerへ置き換わった。この変更はシーケンスごとに効果が
逆であり、共有outdoor presetを一律に旧挙動へ戻すことはできない。

| シーケンス / 設定 | chkpt | RMSE (m) | median | max |
|---|---:|---:|---:|---:|
| seq2 / modern, range 100 | 19/19 | 1.957 | 0.727 | 5.702 |
| seq2 / legacy voxel order, range 100 | 19/19 | 1.652 | 0.634 | 4.795 |
| **seq2 / legacy voxel order, range 105** | **19/19** | **0.426** | **0.264** | **1.246** |
| seq1 / modern, range 100 | 36/36 | 0.838 | 0.511 | 2.468 |

従ってseq1は共有outdoor presetのmodern samplerを維持し、seq2だけ
`configs/mid360_robot/rko_lio_rtk_slam_mid360_stadtgarten_seq2.yaml` を使う。
`legacy_voxel_downsample` は既定falseの互換optionであり、他のデータセットの
軌跡は変えない。seq2公式bagはLiDARがIMUより約58秒長いため、完全再生時は
`--completion-end-margin-secs 65` を指定する。

## ゲート構成(v0.5)

- 屋内 2 profile（`mid360_gt_rtkslam_construction_seq{1,2}`）= **blocking**
  に昇格（pass 0.55/0.30、4 シーケンス計測済み）。
- 屋外 Stadtgarten pair = report-only で soak（実測 0.835 / 1.666 m に
  indoor と同程度のヘッドルームで pass 1.20 / 2.20、target 0.90 / 1.70）。
  屋外は公開手法(GNSS/loop closure 込み)に大差があり、ゲートというより
  能力境界の正直な計測値。長尺 1km 周回(seq1)では LiDAR-inertial raw
  odometry のドリフトが median 1.5 m まで積算する。
- `mid360_vs_glim` = report-only に降格(D-GT-2)。cross-validation は
  agreement しか測れないため、同一センサの回帰カナリアとして残置。

## 帰属(CC-BY 4.0)

データ出典: RTK-SLAM Dataset, Zhang, Ress, Skuddis, Soergel, Haala
(University of Stuttgart), arXiv:2604.07151, CC-BY 4.0. 本リポジトリは bag と
GT を再配布せず、ダウンロードスクリプト経由で利用者が直接取得する。
