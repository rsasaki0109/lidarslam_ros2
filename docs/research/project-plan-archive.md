# lidarslam-ros2 総合計画書（履歴アーカイブ）

> この文書は2026年前半の開発記録です。現在の計画とリリース状態は
> [`docs/roadmap/v0.9.md`](../roadmap/v0.9.md) と
> [`docs/v1-readiness.md`](../v1-readiness.md)
> を参照してください。

## 1. プロジェクト概要

### ゴール
MIT/BSD ライセンスで、Autoware ユーザーが使える高品質な LiDAR SLAM マッピングツール。
- GPL 汚染なし（商用利用可能）
- Autoware の `pointcloud_map_loader` 互換の PCD マップ出力
- GNSS 連携による地理座標系マッピング
- RKO-LIO フロントエンド + graph_based_slam ループクロージャーバックエンド

### 現在の状態
**v0.5 完了（2026-06-11, PR #228）**: RTK-SLAM 公開データで MID-360 を真の
total-station GT 化（§11）— 4/4 sequence 計測済み、indoor 2 profile は **blocking**
昇格、outdoor pair は専用 preset（`double_downsample: false`）付きで report-only soak、
`mid360_vs_glim` は report-only canary に降格。

配布面も 2026-06-11 に大きく前進（§13）: ghcr Docker イメージ初公開（README の
ワンコマンドが実際に動く）、Autoware マップバンドルの lanelet2 自動生成完成
（bag 1 コマンド → pointcloud_map + projector_info + lanelet2、PR #233）、
バージョン 0.5.0 整列 + rosdistro (bloom) バイナリリリース prep。README の見せ物は
実カメラ色のマップフライスルー GIF（§12、PR #229/#230/#231）。

**v0.6 完了（2026-06-12, PR #237–#263, tag v0.6.0）**: 決定論 core/shell
リファクタリング完遂。same bag ⇒ byte-identical map（backend +
scanmatcher frontend の両オフラインランナー、リリースゲートで強制）、
event-driven loop search default 化、実バグ修正（~6% 入力ドロップ /
IMU/GNSS 誤ブロック重み / tie-break 欠落 / GNSS yaw 整合 = 初の
georeferenced マップ）、god object 分解（純ヘッダ 19+4 本、テスト 1100 超）。

**v0.7 完了（2026-06-13, PR #264–#279）**: オフラインマップリファインメント
+ マップ品質ゲート（map authoring の SOTA、[`docs/roadmap/v0.7.md`](../roadmap/v0.7.md)）。
Phase 0 legacy timer 削除（#265、md5 不変で挙動保存証明）→ Phase 1 凍結
マップ品質メトリクス（MME/平面厚/coverage、optimizer が自分の審判を
動かせないよう先に凍結、#266/#267）→ Phase 2 clean-room（BSD-2、論文
のみ、GPL 参照禁止）BALM2/HBA 風 階層平面 BA（#268–#274、6 層ライブラリ +
有限差分・合成 GT オラクル、「間違ったマップを磨く」故障を prior で抑止）
→ Phase 3 holdout 検証付き閾値 + refine default-on（#275–#278、tuning は
cs1+NTU のみ→holdout cs2 を未接触で 5/5 PASS）。**実 GT 3 基盤すべてで
リファインメントが軌跡とマップを同時改善**（APE NTU −3.2% / cs1 −0.7% /
cs2 −1.5%、壁厚 −12.4%/−3.7%/−4.0%、3-run バイト一致）。クロージング
full gate green。発信用技術 writeup（#279、`docs/research/byte-reproducible-map-authoring.md`）。

**進行中: v0.7 Phase 4（stretch）= HILTI 2022 外部 mm-GT 評価基盤**
（§2.4、task #18）。RKO-LIO 生オドメトリを mm 制御点で検証: exp01
construction floor **6.6cm**（優秀）、exp07 long corridor **0.32m**（退化で
約 5 倍ドリフト = v0.8 degeneracy registration の動機づけ）。両者とも非再訪
スイープでループはゼロ（正しい挙動、backend パススルー）。methodology
note: 疎な submap 軌跡は補間でなく submap 時刻で採点すべき（§2.4）。

次の候補: bloom リリース実行（task #14、ndt_omp_ros2 fork 先行、
`docs/rosdistro-release.md`、maintainer の GitHub 操作）/ v0.8 スコープ
（退化検出 / マルチセッション / 動的除去 / intensity / HILTI 拡張）/
発信（v0.6 決定論 + v0.7 リファインメント evidence が素材、1000 スター施策）。

---

## 1.1 追加トラック（2026-04）：KITTI Odometry LO / small_gicp チューニング

2026-04 の LO ベースライン整備＋small_gicp KITTI スイープのハマりどころ（空 TUM の
QoS 不一致 / launch override 順 / 短尺 bag fallback）と再現コマンドは
[`docs/research/kitti-lo-tuning-2026-04.md`](kitti-lo-tuning-2026-04.md)
に集約。

**現状（live）**: 公開推奨は引き続き RKO-LIO + graph_based_slam。KITTI / Velodyne-only
は評価・チューニング用の追加導線。00/05/07 のフルスイープ report は v0.4 §E
（opportunistic）として未実施。

---

## 1.2 追加トラック（2026-05）：STD/BTC 風 Triangle Descriptor 自前実装

13 PR で develop 投入した opt-in triangle descriptor stack（NTU / MID-360 / Newer の
3-dataset ablation、max_pairs=16 sweet spot、RANSAC cost isolation、≥3-run variance
discipline、PR #159-#189）の研究記録は
[`docs/research/triangle-stack-2026-05-summary.md`](triangle-stack-2026-05-summary.md)
に集約。

**現状（live）**: 全プリセットで default-off（opt-in）。3-dataset で variance-bounded、
SOTA は狙わない honest stance。open question は**全決着**: U-shape root cause（wall-clock
floor 仮説）と scheduling fix は v0.4 §D1 で closeout、最後に残っていた 8-vs-16
head-to-head 検証も 2026-06-11 に実施 — `deterministic_loop_scheduling` は機械的には
正しく動くが精度・再現性とも勝たず **default off 維持で完全クローズ**。mp8 の系統的
regression 署名は flag 下で消える（スケジューリングが主要因子であることと整合、
ただし根本原因の証明までは届かない）（research summary 参照）。

---

## 1.3 追加トラック（2026-05）：Dogfood wrapper measurement plumbing (PR #166)

### 目的
- `scripts/run_rko_lio_graph_autoware_dogfood.sh` は「ロボットが撮った bag を SLAM → corrected trajectory → Autoware map verify まで 1 コマンドで通す」操作員向け wrapper。これを **実環境の bag (frame name が launch default と違う / 長尺で /map_save 後も graph_based_slam が submap を処理し続ける) で安定動作させる**ために、計測系の plumbing を拡充した。

### 投入した PR
- **PR #165 (`129eb58`)** — `path_to_tum.py` / `odom_to_tum.py` の custom signal handler を削除。`rclpy.spin()` 中は Python signal handler が走らず、`kill -INT` で hang していた。rclpy の default SIGINT handler に委譲 + `KeyboardInterrupt` / `ExternalShutdownException` を catch して finally で `rclpy.try_shutdown()`。dogfood pipeline で観測された「`path_to_tum.py` subscriber が `Map outputs saved` 後も 40 分以上生き残る」問題を解決。
- **PR #166 (`5929728`)** — dogfood wrapper measurement plumbing 本体：
  - **frame override flags**: `--base-frame`, `--lidar-frame`, `--imu-frame` を追加。robot の frame name が launch default と異なるケース対応。
  - **quiescence-based offline completion**: `--offline-quiet-log-secs` を追加。RKO-LIO offline node の stdout に N 秒間ログが出なければ完了と判定。
  - **graph-drain wait**: `--graph-drain-secs` で、`/map_save` 前に graph_based_slam が buffered submap を消費し終えるまで待つ。長尺 bag で submap 残り処理中に `/map_save` が走って map が incomplete になる問題への対策。
  - **/modified_path → traj_corrected.tum 取込み**: `--capture-corrected-path`, `--corrected-path-topic` を追加し、ループクロージャ補正後の trajectory を録る。
  - **APE vs reference**: `--reference-tum FILE` で reference TUM 軌跡を渡せば evo APE を計算して `traj_corrected_ape.txt` を吐く。
  - **path_to_tum subprocess reap**: cleanup() で structured kill (SIGINT 先行 → 10s 後 SIGKILL guard)。

### Why
- dogfood wrapper が **production candidate session (PR #176)** の上流レイヤとして使われるため、frame 不一致 / offline 完了判定 / graph-drain / corrected path capture の 4 系列をすべて wrapper の責務に閉じ込めた。これにより `run_mid360_robot_production_candidate_session.sh --run` が wrapper を呼ぶときに、bag → corrected trajectory → APE まで一気通貫で取れる。
- PR #166 の measurement plumbing がなければ MID-360 chain (PR #168-#177) は意味のある "production-readiness" を gate できない。

---

## 1.4 追加トラック（2026-05）：README 操作員向け書き直し (PR #163/#164/#167)

### 目的
- README が status / scope の jargon-dense な metadata block で始まり、showcase 画像が line 97 まで埋まっていた。新規訪問者が「このリポジトリで何ができるか」を 1 スクロールで掴めない状態。
- 3 段階のリライトを経て **plain technical README @ ~110 行 / 5 分 quickstart / docs/ への deeper link table** に着地した。

### 投入した PR
- **PR #163 (`2926e92`)** — initial rewrite: badges + hero showcase 画像 + **'5 Minutes' quickstart** (clone → build → quickstart → `map_verify: PASS` 確認) + themed grouping (🗺️ Mapping, 🚗 Autonomous Driving, 🔁 Loop Closure, 📊 Benchmarks, 🧰 Operator Tooling) + docs category table。
- **PR #164 (`a5498bd`)** — tone-down: emoji / horizontal rules / "5 Minutes" framing / hero block を削除し plain technical README に。157 行（220 行 cap 内）。 `test_docs_entrypoints.py` の全 assertion を維持。
- **PR #167 (`dfeb2ab`)** — final simplify: 必須トピック table / dynamic-object-filter figure / 4 つの benchmark CLI 例を `docs/workflows.md` + `docs/benchmarking.md` に逃がす。README order を Install → Quickstart → "Use your own bag" → Features に。docs link を **Getting started / Pipelines / Benchmarking / Project** で grouping。最終的に ~110 行。

### 結果
- README は 1 スクロールで主要情報が読める長さに到達。
- `test_docs_entrypoints.py` で記載項目が継続的に gate されており、deeper detail が docs/ に逃げても link 健全性は保たれる。
- Phase 2 chain (#171-#177) を land する直前に README が片付いていたので、操作員向け entrypoint table (`scripts/run_mid360_robot_production_candidate_session.sh`, `scripts/import_mid360_robot_production_candidate_bundle.py`) を後で追加するときに整理しやすい状態を確保した。

---

## 2. ベンチマーク結果

### 2.1 Newer College math-hard (320m, Ouster OS0-128, IMU あり)

#### LIO + ループクロージャー

| 順位 | 手法 | RMSE (m) | ライセンス | 備考 |
|------|------|----------|-----------|------|
| 1 | DLIO | 0.070 | MIT | 最良精度だが DDS 問題で他ノードと共存不可 |
| 2 | **RKO-LIO + loop closure** | **0.078** | MIT | graph_based_slam, info=1000, Scan Context |
| 3 | RKO-LIO raw | 0.082 | MIT | ループ補正なし |

#### LO (LiDAR-Only)

| 順位 | 手法 | RMSE (m) | ライセンス | 備考 |
|------|------|----------|-----------|------|
| 1 | GenZ-ICP (tuned) | 0.112 | MIT | planarity=0.5, deskew=true, 再現性にバラつき |
| 2 | KISS-ICP | 0.440 | MIT | 安定、リファレンス |
| 3 | lidarslam NDT baseline | 24.286 | BSD | 元の baseline |

### 2.2 NTU-VIRAL tnp_01 (580s, Ouster OS1-16, VN-100 IMU)

| 手法 | RMSE (m) | ループ | 備考 |
|------|----------|--------|------|
| RKO-LIO raw | 1.246 | - | |
| **RKO-LIO + loop closure** | **0.869** | 1回 | 30% 改善 |
| RKO-LIO + loop closure (14回) | **1.314** | 14回 | 検証実行 |

### 2.3 MID-360 (277s, Livox MID-360, 内蔵 IMU, vs GLIM 参照)

| 手法 | RMSE vs GLIM (m) | ループ | 備考 |
|------|-------------------|--------|------|
| RKO-LIO raw | 10.3 | - | |
| RKO-LIO + loop closure (best) | **4.00** | 1回 | info=100, threshold=15.0 |

**MID-360 の限界**: 非360 FOV のため Scan Context 無効、中間ドリフトの補正にループが不足。

#### 2.3b MID-360 実 GT（RTK-SLAM dataset, total-station checkpoints, 2026-06）

GLIM agreement に代わる**真の total-station GT**（§11）。自前 RKO-LIO の dense odometry を
SE(3)-aligned checkpoint RMSE で採点（4/4 sequence 計測済み、2026-06-11 完了）：

| sequence | 環境 | RMSE (m) | median | gate | 公開 (fast_lio / okvis) |
|---|---|---|---|---|---|
| construction_seq2 | indoor hall | **0.154** | 0.061 | blocking, pass 0.30 | 0.086 / 0.075 |
| construction_seq1 | indoor hall (最難) | **0.403** | 0.263 | blocking, pass 0.55 | 0.221 / 0.227 |
| stadtgarten_seq2 | outdoor park | **0.835**（outdoor preset） | 0.327 | report-only | 0.070 / 0.083 |
| stadtgarten_seq1 | outdoor park, 1.04 km | **1.666** | 1.511 | report-only | 0.071 / 0.054 |

indoor で 0.15–0.40m（真 GT, agreement でなく accuracy）。outdoor の当初 3.903m は
**correspondence starvation** で、`double_downsample: false` のみで 0.835m に改善
（粗 voxel 1.0m 案は 2.348m で却下）。outdoor preset:
`configs/mid360_robot/rko_lio_rtk_slam_mid360_outdoor.yaml`。詳細・経緯・profile は §11。

#### 2.4 HILTI 2022（Hesai PandarXT-32, Alphasense IMU, mm 制御点 GT）

外部 mm 級 GT 評価基盤（v0.7 Phase 4、CC BY-NC-SA、評価専用・再配布なし）。
公開チャレンジの total-station 制御点（疎、静止時取得）に対し dense raw
odometry を `ape_from_tum.py --interpolate` で採点（download:
`scripts/download_hilti2022.sh --sequence exp01|exp07`、
config: `configs/hilti2022/rko_lio_hilti2022_pandar.yaml`、voxel 0.25 / deskew on）：

| sequence | 環境 | RKO-LIO raw RMSE (m) | median | path | pairs | loops |
|---|---|---|---|---|---|---|
| **exp01** | construction ground level | **0.066** | 0.053 | 一筆書き | 13 | 0（非再訪）|
| **exp07** | long corridor（退化）| **0.318** | 0.329 | 124m 一筆書き | 6 | 0（非再訪）|

exp07 は廊下退化シナリオ。raw 0.32m は exp01（6.6cm）の約 5 倍 = 特徴の
乏しい廊下軸での測定可能なドリフト（~0.26% of path、壊滅的ではない）。
per-axis 残差 x=0.145 / y=0.235 / z=0.157。両シーケンスとも非再訪一筆書きで
ループはゼロ（backend パススルー）。**これが v0.8 degeneracy-aware
registration の動機づけ基盤**（退化方向の情報行列縮退を検出して
registration を安定化）。

exp01 の corrected 採点メモ（参考）:

| 手法 | RMSE (m) | pairs | 備考 |
|---|---|---|---|
| corrected（submap, sparse） | 0.891 | 12 | スコアリング・アーティファクト（下記）|

**所見**: exp01 は非再訪の一筆書きスイープ（904m / 228s）でループ閉じ込みは
**ゼロ**（max edge `|i-j|`=5 = 隣接拘束のみ、IMU/GNSS/prior 全て無し）。これは
正しい挙動。pose graph 最適化は完全な **no-op**: submap タイムスタンプで
corrected == raw オドメトリが diff 0.000m で一致（検証済み）。corrected の
0.89m は**実劣化ではなく採点アーティファクト** — GT 制御点は静止時取得のため
distance-threshold で submap が作られない時間帯に落ち、最寄り submap から
2.3–7.6s 離れる。手持ち ~1m/s でこの間に数 m 動くため、疎な submap 軌跡
（中央 1.2s, 最大 19.9s 間隔）の線形補間が誤差を生む。**結論: 短く非再訪な
シーケンスでは backend は忠実なパススルー、deliverable は dense odometry
（6.6cm）。疎な corrected/submap 軌跡は補間でなく submap 時刻で採点すべき**
（評価ハーネスの methodology note → 2026-07-05 実装済み: `ape_from_tum.py
--sparse-match`、棄却診断付き。`test_sparse_trajectory_scoring.py` で
補間アーティファクト ~5m vs 正採点 ~0.05m を合成実証）。extrinsic は calibration の quaternion を
`[x,y,z,w]` として読むのが正（`[w,x,y,z]` 解釈は z が −229m に発散、経験確定）。

> 注（2026-06-13 訂正）: 当初「デフォルトループの過剰発火で corrected 悪化」と
> 記録したが、pose graph 直接検査でループ・IMU・prior が全てゼロと判明。劣化は
> 存在せず、0.89m は疎サンプリング採点のアーティファクトだった。

---

## 3. 実装済み機能

### 3.1 graph_based_slam 改善

| 機能 | 状態 | 説明 |
|------|------|------|
| Odometry 直接入力モード | ✅ | `use_odom_input` で RKO-LIO/DLIO の Odometry を直接受信 |
| Cloud-driven サブマップ生成 | ✅ | Odom + Cloud の同期サブマップ作成 |
| GPL フリー Scan Context | ✅ | IROS 2018 論文からフルスクラッチ実装 |
| BSD-2 Triangle Descriptor stack | ✅ (opt-in) | STD/BTC 風 keypoint+hash+RANSAC+SE(3) initial guess を自前実装。default off。詳細は §1.2 |
| BEV mutual-visibility cross-verify | ✅ (opt-in) | triangle 候補を BEV mutual visibility distance で AND ゲート |
| Robust kernel 切替 | ✅ | Huber / DCS / Cauchy をパラメータで切替（`loop_edge_robust_kernel_type`） |
| PCD ディスクキャッシュ | ✅ | OOM 対策、サブマップを逐次 PCD 保存 |
| 情報行列バグ修正 | ✅ | ループエッジを固定重み、オドメトリエッジに `adjacent_edge_info_weight` |
| 隣接エッジ情報重み auto-scale (Level 1) | ✅ (opt-in) | NIS median トラッキングで `adjacent_edge_info_weight` を EMA 自動調整 |
| IMU 回転制約 | ✅ | ジャイロ積分でロール・ピッチ制約 |
| GNSS 位置制約 | ✅ | NavSatFix → ENU 変換 → ユナリエッジ (未テスト) |
| Autoware グリッド PCD 出力 | ✅ | `pointcloud_map_metadata.yaml` + 分割 PCD (検証済み) |
| `map_projector_info.yaml` | ✅ | GNSS 原点の地理座標出力 (未テスト) |

### 3.2 scanmatcher 改善

| 機能 | 状態 | 説明 |
|------|------|------|
| 非単調タイムスタンプスキップ | ✅ | ROS2 bag 再生時の時刻逆転対応 |
| VoxelHashMap | ✅ | KISS-ICP 着想のボクセルマップ |
| 適応閾値 | ✅ | EMA ベースの correspondence distance 自動調整 |
| FAST_GICP / SMALL_GICP | ✅ | オプショナル依存 (`#ifdef` ガード) |
| cloud_queue_depth | ✅ | キュー深度パラメータ化 |

### 3.3 インフラ

| 機能 | 状態 | 説明 |
|------|------|------|
| `rko_lio_slam.launch.py` | ✅ | RKO-LIO + graph_based_slam 統合ランチファイル |
| `verify_autoware_map.py` | ✅ | Autoware 互換性検証スクリプト |
| `odom_to_tum.py` / `path_to_tum.py` | ✅ | 軌跡ロギングツール |
| CI ローカルビルド | ✅ | 全パッケージビルド + テスト 25/25 パス |
| README | ✅ | ベンチマーク結果、Autoware 使い方、パラメータ一覧 |

---

## 4. 各手法の深掘り分析

### KISS-ICP — なぜ LO 系で安定か

- **VoxelHashMap**: tsl::robin_map で O(1) ルックアップ、sub-voxel 距離チェック
- **27近傍探索**: 3x3x3 ボクセルキューブ、KDTree 不要
- **Robust kernel**: `w = σ² / (σ² + r²)` で外れ値自動排除
- **Adaptive threshold**: motion model error RMS で `τ = 3σ`
- **Constant velocity prediction**: 収束が速い
- **処理速度**: 20-30 fps、共分散計算なし

### GenZ-ICP — チューニング結果

- **最良設定**: `voxel_size=0.5, planarity=0.5, deskew=true`
- **結果**: RMSE 0.112m (KISS-ICP の 0.440m を大幅に上回る)
- **問題**: 再現性にバラつき (0.112〜0.146m)、rate や DDS 状態に依存
- **voxel_size=0.4 以下は劣化**、0.6 以上は発散

### DLIO vs RKO-LIO

| 要素 | DLIO (0.070m) | RKO-LIO (0.082m) |
|------|---------------|-------------------|
| IMU 統合 | Jerk ベース 3次連続モデル | 定加速度 + カルマンフィルタ |
| デスキュー | 各点ごとの SE(3) 補間 | フレーム境界間の補間 |
| マッチング | NanoGICP (共分散あり) | カスタム point-to-plane ICP |
| マップ | キーフレーム + 凸/凹ハル | Bonxai 疎ボクセルグリッド |
| **問題** | **DDS メッセージ遅延で他ノードと共存不可** | **安定、offline_node で統合成功** |

---

## 5. ライセンス調査

### 使える (MIT/BSD + ROS2 対応)

| 手法 | 分類 | ライセンス |
|------|------|-----------|
| KISS-ICP | LO | MIT |
| GenZ-ICP | LO | MIT |
| small_gicp | 登録ライブラリ | MIT |
| DLIO | LIO | MIT |
| RKO-LIO | LIO | MIT |

### ライセンス NG

| 手法 | ライセンス |
|------|-----------|
| FAST-LIO2 / Faster-LIO | GPLv2 |
| LIO-SAM | BSD だが GTSAM Jazzy 互換問題 |
| LiLi-OM / MULLS / MOLA | GPL |

---

## 6. Autoware 対応状況

### 検証済み ✅

| 項目 | 状態 | 詳細 |
|------|------|------|
| グリッド分割 PCD | ✅ PASS | 20x20m セル、binary_compressed |
| `pointcloud_map_metadata.yaml` | ✅ PASS | `filename.pcd: [int, int]` 形式、Autoware の yaml-cpp パーサー互換 |
| PCD ヘッダー | ✅ PASS | v0.7, FIELDS x y z intensity, float32 |
| orphan ファイル防止 | ✅ PASS | 出力前にディレクトリクリーンアップ |
| `map` フレーム座標系 | ✅ | REP-105 準拠 |

### 未検証 ⚠️

| 項目 | 状態 | 理由 |
|------|------|------|
| GNSS ポーズグラフ制約 | 🟡 部分実証 | 2026-06-12 RTK-SLAM stadtgarten_seq2 で初検証: #242 修正後にエッジ 163 本形成・RTK 判定・ENU 変換まで動作 ✅。odom→ENU 初期 yaw 整合も PR #244 で実装済み（2D Kabsch + vertex-0 gauge 解放、stadtgarten_seq2 で実証）。詳細: `docs/research/gnss-constraint-first-validation.md` |
| ~~`map_projector_info.yaml`（LocalCartesian）~~ | ✅ | 同検証で初実証: LocalCartesian + WGS84 + 実origin（Stadtgarten 48.78199/9.17295）が出力された |
| ~~Autoware 実環境読み込み~~ | ✅ | map loaders 読込 dogfood 済み |

### Autoware ユーザーへのバリュー

1. **MIT ライセンスの SLAM** — LIO-SAM (GPL) の代替として商用利用可能
2. **ループクロージャー付き高品質マップ** — ドリフト補正済み PCD
3. **`pointcloud_map_loader` 直接互換** — 変換ツール不要
4. **GNSS 連携** (実装済み、テスト待ち) — 地理座標系マッピング

---

## 7. 既知の問題と制限

### 7.1 DDS メッセージ遅延
- **影響**: DLIO が他ノードと共存できない、online_node でスキャンドロップ
- **原因**: 大きな PointCloud2 メッセージ (6MB+) の DDS 転送遅延
- **回避策**: offline_node (RKO-LIO) でバッグを内部読み込み（既定パスはこれ）
- **根本解決**: FastDDS のシェアードメモリ設定、またはゼロコピー転送
- ユーザー向けの失敗モード解説・CycloneDDS + kernel チューニング・intra-process
  composition の状況は [`docs/dds-tuning.md`](../dds-tuning.md) に集約（v0.4 §F）。

### 7.2 MID-360 (固体 LiDAR) の限界
- 非 360 FOV のため Scan Context が無効
- 中間ドリフトの補正にループクロージャーが不足
- RMSE 4.0m (vs GLIM) が現状の限界
- **2026-06 実 GT で定量化（§11）**: indoor 構造化環境は良好（construction hall
  0.15–0.40m, 真 total-station GT）だが、**open-outdoor（Stadtgarten park）で 3.9m に
  大ドリフト** — 短距離（~40m）× 疎・遠方特徴で odometry が starve（誤差が 11.7m まで
  単調増加）。**→ 解決済み（2026-06-11）**: 原因は correspondence starvation で、
  `double_downsample: false` のみで 0.835m（outdoor preset
  `rko_lio_rtk_slam_mid360_outdoor.yaml`）。sweep 記録は
  `docs/research/rtkslam-total-station-gt-methodology.md`
- BSD-2 自前実装の STD/BTC 風 triangle descriptor を 2026-05 に投入。NTU VIRAL ablation v4 で初の triangle 採用 (id=32↔95, 補正 0.49m/1.06°)、v5 で 4-point gate + inlier_ratio による偽陽性 emit 半減 (4→2) と distance loop 押し出し解消を確認。詳細は §1.2。default off の opt-in 機能として develop に landing 済。次の段は MID-360 demo bag 整備 → 同じ ablation を MID-360 でも回すこと。

### 7.3 GenZ-ICP の再現性
- DDS のメッセージ配送タイミングに結果が依存
- 同一設定で 0.112m〜26m の幅がある
- offline 実行モードが必要

### 7.4 small_gicp オドメトリの処理速度
- IncrementalVoxelMap の NN 探索が律速
- 共分散計算のオーバーヘッドでスキャンドロップ多発
- ICPFactor への切替で改善可能だが未実装

---

## 8. 今後のアクション候補

### 優先度: 高

| # | タスク | 理由 |
|---|--------|------|
| 0a | ~~v0.5 outdoor config 調整 → stadtgarten 再 run~~ | ✅ 2026-06-11 解決。`double_downsample: false` で 3.903→0.835m、outdoor preset 化（§11.8 A） |
| 0b | ~~v0.5 indoor pair を blocking 昇格 + mid360_vs_glim 降格~~ | ✅ PR #228。4/4 seq 計測、indoor blocking、glim は report-only canary（§11.8 C） |
| 0c | **bloom リリース実行（P2-7 後半）** | repo 側 prep 完了（0.5.0 整列 + ランブック `docs/rosdistro-release.md`）。残り = ndt_omp_ros2 fork PR #11 マージ（整備 PR 作成済み）→ 先行 bloom → 本体 bloom → ros/rosdistro PR（maintainer の GitHub 操作が必要、§13） |
| 0f | ~~v0.6 決定論 core/shell リファクタリング~~ | ✅ 2026-06-12 完遂（PR #237–#263、tag v0.6.0 + pre-release）。全フェーズゲート PASS: 両基盤 3-run バイト一致（backend + frontend）、event-driven default 化、純ヘッダ 19+4 本。詳細 [`docs/roadmap/v0.6.md`](../roadmap/v0.6.md)、[`docs/releases/v0.6.0.md`](../releases/v0.6.0.md) |
| 0g | ~~v0.7 マップリファインメント + 品質ゲート~~ | ✅ 2026-06-13 完遂（PR #264–#279）。Phase 0–3 全 PASS: clean-room 階層平面 BA、holdout 検証済み blocking 閾値、refine default-on、実 GT 3 基盤で APE+マップ同時改善、クロージング full gate green。詳細 [`docs/roadmap/v0.7.md`](../roadmap/v0.7.md)、[`docs/research/byte-reproducible-map-authoring.md`](byte-reproducible-map-authoring.md) |
| 0h | **v0.7 Phase 4: HILTI 2022 外部 mm-GT 基盤（進行中）** | exp01 6.6cm / exp07 long corridor 0.32m（退化、§2.4、task #18、PR #280/#282）。両者ループゼロで backend パススルー（正常）。残り = 他 construction 系列 / sparse corrected 軌跡の採点改善 / **exp07 退化を v0.8 degeneracy registration の検証基盤に** |
| 0d | ~~D1 8-vs-16 再現性ベンチ~~ | ✅ 2026-06-11 実施（GLIM MID-360 bag、3 run × {off/16, on/16, on/8}）。on/16 は APE 実質無回帰（差 0.06m ≪ noise 0.40m）だが分散縮小なし（σ 0.066→0.259）、on/8 arm の 2 実行で loop 試行ゼロ。mp8 の系統的 regression 署名は消滅し乱高下に置換（スケジューリング主要因子と整合、根本原因の証明ではない）。**default off 維持で D1 完全クローズ**（§1.2、research summary） |
| 0e | **発信（P2-8）** | ghcr ワンコマンド + lanelet2 完全バンドル + 実 GT 数値が揃い、発信material は完成状態。ROS Discourse / Reddit / X はユーザー本人が実施。事前に B2 social preview 設定（Web UI 1 分） |
| 1 | ~~GNSS odom→ENU yaw 整合の実装~~ | ✅ 実装済み（PR #244 `18fdbc1`、2026-06-12）。閉形式 2D Kabsch（`gnss_alignment.hpp`）+ vertex-0 gauge 解放方式。最小観測性ガード（`gnss_yaw_alignment_min_anchors`=10 / `gnss_yaw_alignment_min_baseline_m`=5.0）付き。stadtgarten_seq2 実データで yaw −111.83° / baseline 201.9m / rms 1.90m を確認済み。2026-07-05 再検証: build + 1379 tests 0 fail。`docs/research/gnss-constraint-first-validation.md` |
| 2 | ~~Autoware 実環境での読み込みテスト~~ | ✅ map loaders 読込は検証済み（README の loader proof、§6） |
| 3a | ~~MID-360 robot toolkit chain (操作員 pipeline)~~ | ✅ PR #168-#177 で 10 PR inside-out で landing 完了 (§10) |
| 3b | **実機 Jetson + MID-360 robot での dogfood 実走** | chain (§10) を組んだものの実機 bag での E2E 検証はまだ。dogfood-vs-bench の cloud distribution 不一致も併せて調査 |
| 3c | Jetson host readiness preflight PR | §10.5 残課題の自然な次。`check_jetson_mid360_host_readiness.py` + `jetson_mid360_host_tools.py` を 1 PR で land |

### 優先度: 中

| # | タスク | 理由 |
|---|--------|------|
| 4 | ~~Triangle keypoint 抽出質改善~~ | ✅ PR #145 v4 default に landing 済、初の採用ループ確認 |
| 4b | ~~4 点以上 consensus への拡張~~ | ✅ PR #147 で実装、v5 で偽陽性半減確認 |
| 4c | MID-360 demo bag の整備 | reference 軌跡 + 短距離ループありの bag が無いと triangle ablation を MID-360 で回せない |
| 4d | 別データセットで triangle stack 再現性検証 | NTU 単独では PoC 段階。Newer College / Leo Drive / MID-360 demo で同じ ablation を回したい |
| 4e | 4-point quad-hash (#161) + N-point refinement (#159) + precision floor (#162) 組合せ ablation | §1.2 の延長線。3 つの knob を組み合わせた最適 emit/accept 比率を測る |
| 4f | preflight 系 (`preflight_mid360_robot_bag.py`, `validate_mid360_robot_profile.py`, `rewrite_mid360_robot_bag_stamps.py`) を land | §10.5 残課題。Jetson host readiness の次の順序 |
| 4g | public_dataset 系 (~15 scripts: download / segments / loop_candidates / dataset_report) を land | §10.5 残課題。public bag の準備を独立 PR で済ませる |
| 4h | 3DGS visual QA/export track を設計・PoC | loop alignment / map split を operator が確認しやすい 3D artifact にする。core SLAM gate ではなく dashboard/bundle の optional artifact として扱う |
| 5 | Robust kernel 導入 | 誤ループ検出への頑健性（既に DCS/Cauchy/Huber 切替は実装済） |
| 6 | キーフレーム選択ロジック | フロントエンドの品質指標に基づくサブマップ生成 |
| 7 | マルチセッションマッピング | 複数回走行データの統合 |

### 優先度: 低

| # | タスク | 理由 |
|---|--------|------|
| 8 | GTSAM 移行 | Jazzy での boost→std 互換問題の解決待ち |
| 9 | DLIO 統合 | DDS 問題の根本解決が先 |
| 10 | small_gicp オドメトリ高速化 | KISS-ICP / RKO-LIO が十分高精度 |
| 11 | docs/jetson-mid360-robot-{runbook,scope,static-tf-worksheet}.md を mkdocs に組込み | §10.5 残課題。実機セットアップ手順をまとめる時に必要だが、§10.4 の codebase 側 land が先 |

---

## 9. 技術的知見

### ループクロージャーのパラメータチューニング

| パラメータ | Newer College 推奨 | MID-360 推奨 | NTU-VIRAL 推奨 |
|-----------|-------------------|-------------|---------------|
| adjacent_edge_info_weight | 1000.0 | 100.0 | 1000.0 |
| threshold_loop_closure_score | 3.0 | 15.0 | 3.0 |
| distance_loop_closure | 100.0 | 100.0 | 100.0 |
| use_scan_context | true | false (非360 FOV) | true |
| scan_context_threshold | 0.3 | - | 0.3 |

**知見**: `adjacent_edge_info_weight` はデータセットの LIO 精度に依存。高精度 LIO (RKO-LIO on Newer College) では 1000 でオドメトリ重視、低精度時 (MID-360) では 100 でループ重視。

### Autoware マップフォーマット

```yaml
# pointcloud_map_metadata.yaml (Autoware 互換)
x_resolution: 20.0
y_resolution: 20.0
-80_-40.pcd: [-80, -40]    # 座標は整数必須 (yaml-cpp as<int>)
-60_-60.pcd: [-60, -60]

# map_projector_info.yaml (GNSS 原点)
projector_type: local
vertical_datum: WGS84
map_origin:
  latitude: 35.6812362
  longitude: 139.7671248
  altitude: 40.0
```

### 重要ファイル

| ファイル | 説明 |
|---------|------|
| `graph_based_slam/src/graph_based_slam_component.cpp` | バックエンド本体 |
| `graph_based_slam/include/graph_based_slam/scan_context.hpp` | GPL フリー Scan Context |
| `graph_based_slam/include/graph_based_slam/triangle_descriptor.hpp` | BSD-2 三角形 descriptor primitives（§1.2） |
| `graph_based_slam/include/graph_based_slam/triangle_descriptor_database.hpp` | hash DB + RANSAC findLoopCandidate（§1.2） |
| `graph_based_slam/include/graph_based_slam/bev_mutual_visibility.hpp` | FOV-aware BEV mutual visibility（triangle cross-verify でも利用） |
| `graph_based_slam/include/graph_based_slam/loop_edge_robustifier.hpp` | Huber / DCS / Cauchy 切替ヘルパ |
| `graph_based_slam/include/graph_based_slam/adjacent_edge_auto_scale.hpp` | NIS median ベースの adjacent edge info weight auto-scale |
| `scanmatcher/src/scanmatcher_component.cpp` | フロントエンド本体 |
| `scanmatcher/include/scanmatcher/voxel_hash_map.hpp` | VoxelHashMap |
| `lidarslam/launch/rko_lio_slam.launch.py` | RKO-LIO 統合ランチ |
| `scripts/verify_autoware_map.py` | Autoware 互換性検証 |
| `scripts/odom_to_tum.py` | 軌跡ロギング |
| `scripts/run_triangle_ablation.sh` | triangle on/off ablation を 1 コマンドで（§1.2） |
| `scripts/generate_place_recognition_report.py` | scan_context / BEV / SOLiD / triangle の loop 採用統計を md/JSON/SVG 化 |

---

## 10. MID-360 robot toolkit chain — 操作員向け session pipeline (2026-05)

inside-out 戦略で 10 PR（#168-#177）投入した操作員向け session pipeline
（foundation / analyzer / dashboard / record / readiness / public-RKO / session /
bundle layer）+ ament lint の罠 + 重要ファイル + 3DGS QA candidate の詳細は
[`docs/research/mid360-robot-toolkit-2026-05.md`](mid360-robot-toolkit-2026-05.md)
に集約。

**現状（live）**: develop に landed（81 mid360 scripts + runbook smoke test +
continuous kidnap-relocalization gate #194）。残りは実 GT データセット
（v0.4 roadmap §C → v0.5、§11 で実現）。

---

## 11. 追加トラック（2026-06）：RTK-SLAM 公開データで MID-360 実 GT

**status: 完了（2026-06-11, PR #228 で v0.5 roadmap done）**

v0.4 の locked 決定 #1「MID-360 evidence は実 GT を狙う」を v0.5 で具体化したトラック。
これまで MID-360 の唯一の release gate は `mid360_vs_glim`（GLIM SLAM 推定との
cross-validation, `ape_rmse_vs_reference_m`, pass 4.00）で、これは accuracy ではなく
*agreement* しか測れない（両系統が共有する系統誤差は不可視）。これを**独立 GT 付き
データセット + `ape_rmse_gt_m`**（NTU VIRAL / Newer College と同じ土俵）に置換するのが
v0.5 の背骨。詳細な live ドキュメントは [`docs/roadmap/v0.5.md`](../roadmap/v0.5.md)。

### 11.1 経緯と再 scope
- 当初 D-GT-1 を「robotic total station の自前撮影」に決定（屋内 loop を保つ最強 GT）。
- 「公開データから探す」方針に転換 → **RTK-SLAM Dataset（Univ. Stuttgart ifp,
  arXiv:2604.07151, 2026, CC-BY 4.0）**を発見・検証。**Livox MID-360 + geodetic
  total station GT（onboard RTK は system input、GT は独立）**、ROS1+ROS2 bag、
  IMU `/livox/imu` 200Hz、182GB、4 seq。→ D-GT-1 を「この公開データで取得」に再 scope、
  **自前 total station / 撮影は不要化**（自前屋内 loop は optional な後続 complement に降格）。

### 11.2 重要発見：GT は 182GB の中に無い
- survey checkpoint は **eval repo（github.com/Willyzw/rtk-slam-eval）の
  `ground_truth/<seq>.csv`（~1-2KB）**に同梱。公開 SLAM の例 trajectory
  （fast_lio_sam / okvis の TUM）も同梱。→ **数MB の git clone だけで reference
  pipeline 全体を実データ検証可能**、多GB bag は自前 RKO-LIO 実行時のみ必要。
- topic 確定: **`/livox/points`（sensor_msgs/PointCloud2）** が SLAM 入力。
  `/livox/lidar` は Livox CustomMsg。checkpoint CSV `timestamp` は Unix-epoch
  sensor clock（bag / GT / 例 trajectory が共通 clock）。bag に `/tf_static` は無く、
  base-center extrinsic は eval の `transform_imu_to_base.py` 側にあるが、
  **SE(3)-aligned metric では alignment が定数オフセットを吸収するため moot**
  （identity extrinsic で可）。

### 11.3 パイプライン（インフラはほぼ再利用）
- **`scripts/generate_rtk_slam_reference.py`（新）**: checkpoint CSV
  （`point_id,easting,northing,height,env,timestamp`）を header 名でマップ（列順
  非依存）→ 最古 checkpoint を local origin に減算（UTM 大座標対策）→ identity quat の
  sparse TUM。source slug に `gt` を含むので `_infer_reference_kind` が ground_truth
  判定（explicit kind 不要）。
- **採点は既存 `write_aligned_trajectory_metrics.py` を無改修で再利用**:
  `_rigid_align` は SVD 前 centroid 減算（UTM 安全）、timestamp tolerance match、
  出力 `alignment: se3_umeyama` の `ape.rmse` = dataset の "SE3" checkpoint metric。
- **`--match-tolerance` フラグを追加**（後方互換: default は現行 0.05→0.15s cascade、
  単一値 2.0 で dataset の max_dt 再現）。疎 checkpoint × downsampled trajectory で
  checkpoint が silent-drop されるのを防ぐ。実データで bias 確認（fast_lio offline は
  default で 7/16・13/36 しかマッチ、tol=2.0 で全 checkpoint）。
- **メトリクス方針**: 我々は **SE(3)-aligned checkpoint RMSE** を gate に採用
  （NTU / Newer College の `ape_rmse_gt_m` 定義と一致）。dataset の zero-align 絶対
  RMSE / gap% は GNSS-anchor 必要（RKO-LIO は LiDAR-inertial only）なので context 扱い。
- **疎 checkpoint GT は dense trajectory で採点するのが正**: 最適化済み
  `/modified_path` は submap-node 単位で疎（最大 24.5s gap）で、gap が survey 滞在点
  （checkpoint）に集中して採点不可。dense な RKO-LIO odometry（~10Hz）が全 checkpoint
  にマッチし、公開ベースラインと同じ dense 形なので fair。loop-closure の恩恵は疎
  checkpoint では観測不可（GT の性質であって optimizer の問題ではない）。

### 11.4 自前 RKO-LIO 実機結果（3 sequences, dense odometry vs total-station GT）

config `configs/mid360_robot/rko_lio_rtk_slam_mid360.yaml`（identity extrinsic,
deskew off, voxel 0.5）、`--match-tolerance 2.0`：

| sequence | 環境 | paired | 我々の RMSE (m) | median | 公開 (fast_lio / okvis) |
|---|---|---|---|---|---|
| construction_seq2 | indoor hall | 16/16 | **0.154** | 0.061 | 0.086 / 0.075 |
| construction_seq1 | indoor hall (最難) | 16/16 | **0.403** | 0.263 | 0.221 / 0.227 |
| stadtgarten_seq2 | **outdoor park** | 19/19 | **3.903** | 1.366 | 0.070 / 0.083 |

**確立した知見：indoor 強い / open-outdoor 弱い**。indoor-tuned config では構造化された
construction hall で 0.15–0.40m（raw odometry, loop closure なし）を保つが、開けた
Stadtgarten park で 3.9m に大ドリフト — MID-360 の短距離（~40m）× 疎・遠方特徴で
odometry が starve。real capability boundary（19/19 paired で誤差が 11.7m まで単調増加、
metric artifact ではない）。これは §7.2 の MID-360 限界を実 GT で定量化したもの。

### 11.5 release profile（per-sequence, report_only_until v0.6）
NTU（1.0m）と Newer College（0.10m）が難易度で閾値を変えるのと同様、per-sequence 閾値：
- `mid360_gt_rtkslam_construction_seq2`: pass 0.30 / target 0.15（PASS）
- `mid360_gt_rtkslam_construction_seq1`: pass 0.55 / target 0.30（PASS）
- **stadtgarten は gate に入れない**: 4m+ の緩い閾値は `mid360_vs_glim` と同じ
  loose-threshold の誤魔化しなので回避。outdoor config 修正後に追加。
- 両 indoor profile とも report_only_until v0.6（疎 16-checkpoint は high-variance、
  他 seq で確証後に blocking 昇格）。

### 11.6 benchmark の `--offline-timeout-secs`
`run_rko_lio_graph_benchmark.sh` の `wait_for_offline_completion` が 1800s
（30分）ハードコードで、~600s の重シーケンス（graph loop closure が ~0.2x realtime）を
62% で打ち切っていた。CLI flag 化（default 1800 維持）し、5400s 等で full 完走。
打ち切りは **SLAM 発散ではなく compute cutoff**（raw odometry は綺麗な 10Hz で完走、
エラーなし）と判明。

### 11.7 投入した PR
- #209 docs: v0.5 scoping（self-capture）→ #211 docs: 公開 RTK-SLAM dataset へ再 scope
- #210 ci: MID-360 robot runbook smoke test を CI 接続
- #212 feat: `generate_rtk_slam_reference.py` + test（合成 CSV で end-to-end 契約実証）
- #213 feat: download tooling + `--match-tolerance` + 実 GT 例 trajectory baseline 検証
- #214 feat: 自前 RKO-LIO 実走 → construction_seq2 GT profile（PASS）+ config + offline-timeout flag
- #215 feat: construction_seq1 profile + 3-seq indoor/outdoor 結果

### 11.8 残タスク → 全消化（2026-06-11, PR #228）
- **A. outdoor config 調整**: ✅ 解決。3.903m の正体は **correspondence starvation** で、
  `double_downsample: false` 単独で **0.835m**（median 0.327, max 3.05, 19/19 paired）。
  粗 voxel 案（1.0m + 1.0m correspondence distance）は 2.348m で却下。outdoor preset
  `configs/mid360_robot/rko_lio_rtk_slam_mid360_outdoor.yaml`、sweep 記録は
  `docs/research/rtkslam-total-station-gt-methodology.md`。副産物 fix: benchmark の
  offline 完了判定を log quiescence から「raw trajectory が bag 末尾到達」へ変更
  （TF warning spam が log を busy にし timeout budget を食い潰していた）。
- **B. stadtgarten_seq1**: ✅ 計測済み。**1.666m**（median 1.511, 35/36 paired）、
  report-only profile 追加。
- **C. indoor pair blocking 昇格**: ✅ report_only_until 撤去（construction_seq1/2 が
  release-blocking）、`mid360_vs_glim` は D-GT-2 どおり report-only canary に降格、
  `comparison.md` / README accuracy 表 / v0.5 roadmap done 更新済み。
- 採点の再現: dense raw trajectory を `--points-topic /livox/points --match-tolerance 2.0`
  で採点 → `benchmark_summary.py --release-profile` で PASS 確認
  （`_profile_match` が points_topic 一致を要求するので `--points-topic` 必須）

---

## 12. 追加トラック（2026-06）：3DGS photoreal map + マップフライスルー GIF

### 目的と着地点
SLAM 成果物の「映え」を README 先頭で見せる。最終着地は **実カメラ色の点群マップを
サードパーソン追従カメラで 60m 周回するフライスルー GIF**
（`lidarslam/images/map_flythrough_rtkslam.gif`、PR #229 → #230 → #231）。
3DGS 学習マップそのものを飛び回る案は**実測で不成立と確定**させた上での pivot。

### 主要な実測知見（負け筋の確定が資産）
- **gsplat (Apache-2.0) + LiDAR-primed init** が品質の核: koide 近接シーンで
  25.2–25.5dB（LiDAR init +5.5dB、詳細 `docs/research/3dgs-koide-first-light.md`）。
  isuzu（~14dB, motion blur + 視点重複小）/ NTU（~10dB, mono 広域疎）は適性外
  （データ特性支配、視点数 21 倍でも届かない）。
- **学習済み 3DGS の「マップ内移動」は崩壊半径 ~0.4m で不成立**: dolly テスト
  （学習視点の回転固定・並進のみ）で、立ち止まり学習クラスタから ~0.4m 離れると
  confetti 状に崩壊。フィルタ全組合せ（LiDAR 距離 voxel-hash / opacity / size / SH 無効化）
  でも俯瞰・サードパーソンは不可。gaussian の色は地上の学習方向からしか意味を持たない。
  記録: `docs/research/3dgs-trajectory-flythrough-notes.md` 追補 2。
- **pivot: カメラ投影色の点群がフォトリアル代替**: 同期カメラ画像を LiDAR 点群へ投影
  （`build_lidar_init.py --color-transforms`）し、同じ gsplat rasterizer で
  等方 splat 描画 → 任意視点で破綻しない。
- **ロバスト着色（PR #231）**: 点群自身から作る粗 z-buffer でオクルージョン棄却 +
  画像ごと輝度中央値で露出正規化 + 点ごと per-channel median 集約（spec/blur 外れ値除去）
  + 3×3×3 voxel 近傍密度で孤立点除去。視点依存の透け汚れ・濁り・ダストを解消。
  強い色補正は robust 色と相性が悪い（白飛び）→ 控えめ default
  （saturation 1.25 / percentile 0.5–99.8 / gamma 1.0）。
- **カメラワーク**: 一人称（dot soup）と固定ピッチ drone（天井ノイズ壁）は失敗。
  正解は ride point − 水平接線·5.5m + 上方 5.5m の**サードパーソン追従** + 等弧長
  リサンプル（立ち止まり潰し）+ 天井カット（ride z + 2.3m、最近傍 XY 追従）。
  OpenCV 規約の右手系は `right = cross(forward, world_up)`（逆順は 180° roll）、
  up は world-z 必須（学習視点 up はマウント傾き ~20°）。

### ツール（リポジトリ投入済み、テスト付き）
- `tools/gaussian_splatting/render_map_flythrough.py` — 等弧長サードパーソン
  フライスルー renderer（--color-mode {height,rgb}、ミニマップ、loop fade、19 tests）
- `tools/gaussian_splatting/pointcloud_io.py` — `colorize_by_projection_robust()` +
  `drop_sparse_points()`（19 tests、grid 境界 searchsorted 回帰テスト含む）
- 再現: `build_lidar_init.py --color-transforms --color-robust --min-neighbors 4` →
  `render_map_flythrough.py --color-mode rgb`（`docs/3dgs-map-tutorial.md` 成果物例）

### 開発プロセス備考
codex CLI（gpt-5.5 xhigh）をサブエージェント運用（ユーザー指示）: ツールなし・
コンテキスト全貼りでコード生成/レビューを依頼し、Claude 側で保存・検証・GPU 描画。
レビューで実バグ 4 件（empty-mask crash / 近一様色の過増幅 / searchsorted IndexError /
stale tmp）を事前捕捉。一方で**ネット無し実行のため「引用」は捏造があり得る**
（§13 の libg2o 誤指摘で実証）— 事実主張は必ず実地検証する。

---

## 13. 追加トラック（2026-06-11）：配布整備 — ghcr / 完全バンドル / rosdistro prep

1000 スターキャンペーン（820→1000）の「いま選ぶ理由を 1 コマンドで体験させる」配布
トラック。本日 1 セッションで PR #229-#233 + prep ブランチまで投入。

### 13.1 ghcr Docker イメージ初公開
- `docker.yml` の publish トリガーが存在しない `main` ブランチを指す死にトリガーで、
  README のワンコマンドが 404 のままだった（**このリポジトリに main は無い、
  default = develop**）。
- workflow_dispatch で初公開 → 匿名 pull 200 確認 → **PR #232** でトリガーを
  `develop` push に修正、merge push 自体で自動再公開も実証。以後 develop に push する
  たびに `ghcr.io/rsasaki0109/lidar_slam_ros2:{humble,latest}` が更新される。

### 13.2 Autoware マップバンドルの lanelet2 完成（P1-4、PR #233）
- beginner 連鎖（`run_autoware_map_beginner.sh` → `run_autoware_map_from_bag.py` →
  dogfood script）は pointcloud_map + projector_info しか出さず、README の
  「complete bundle」claim に lanelet2 が欠けていた（generator は orphaned script
  からのみ到達可能だった）。
- dogfood script の `/map_save` 後に `traj_corrected.tum` → `lanelet2_map.osm` 生成を
  組込み（`--generate-lanelet2` default true / `--origin-lat/lon` default 0.0 =
  local origin / `--lane-width` 3.5）。quickstart / beginner / **ghcr demo image**
  の全入口が完全バンドルを出すようになった（Dockerfile に python3-scipy 追加が必要
  だった — generator の scipy は rosdep で入らない）。
- 設計判断: 生成は **best-effort**（script 終盤は corrected-path / APE / map_save
  fallback すべて best-effort 設計で一貫。10 分の SLAM 実行を付加成果物で fail させ
  ない）。可視性は**末尾のバンドルサマリ**（OK/MISSING）で担保。stale 対策は
  事前削除 + `.tmp` 書き → 成功時のみ `mv`（generator は構造検証失敗でも書き込み済み
  ファイルを残すため必須）。

### 13.3 バージョン 0.5.0 整列 + rosdistro (bloom) prep
- ドリフト解消: VERSION / 4× package.xml が 0.2.2 のまま CHANGELOG.md と git tag は
  0.3.0 に進んでいた → 全部 **0.5.0**（公開 v0.5 マイルストーンと一致）に整列。
  license タグも SPDX `BSD-2-Clause` 化。整合性テスト
  （`test_release_metadata_and_core_package_versions_match`）が per-package
  CHANGELOG.rst の名前+先頭バージョンまで検証するよう拡張。
- **依存分析の結論**（`docs/rosdistro-release.md` ランブック）:
  - コア 4 パッケージの未リリース依存は **`ndt_omp_ros2` のみ**（自分の fork、BSD、
    rosdistro で名前空き）→ fork の package.xml 整備（0.0.0→0.1.0、maintainer、SPDX）
    後に**先行 bloom リリース**すれば解決。
  - **rko_lio は package.xml 非依存**（flagship launch の実行時のみ）→ バイナリ
    リリースのブロッカーではない。apt ユーザーは classic `lidarslam.launch.py`、
    RKO-LIO はソース/Docker（upstream のリリース判断は我々の管轄外）。
  - `libg2o` は両ディストロでリリース済み（**実地検証**: rosdep resolve →
    jammy `ros-humble-libg2o` / noble `ros-jazzy-libg2o`。codex レビューが
    「未リリース BLOCKER」と誤指摘 — ネット無し環境での記憶ベース引用だった）。
  - submodule は `git archive` に入らないので bloom の upstream import から
    Thirdparty が自然に除外される（意図どおり）。
- **残り（maintainer の GitHub 操作が必要）**: v0.5.0 tag push → ndt_omp_ros2 fork
  整備+bloom → 本体 `bloom-release --rosdistro {humble,jazzy} lidarslam_ros2` →
  ros/rosdistro PR。release tag は `v:{version}`（v-prefix、release.yml の `v*`
  trigger と一致）。初回 sync 後に README へ apt インストール手順を追記。
