# KITTI Odometry LO / small_gicp tuning — 2026-04 アーカイブ

> 2026-04 の追加トラック記録。2026-06-07 の docs/research/project-plan-archive.md surgery（v0.4 roadmap §F）で
> `docs/research/project-plan-archive.md` から抽出した歴史的アーカイブ。LO ベースライン整備と small_gicp KITTI
> スイープのハマりどころ・再現コマンドを保存する。live なステータスは `docs/research/project-plan-archive.md` §1.1。

### 目的
- **KITTI Odometry (Velodyne only, IMU なし)** で動く **LiDAR Odometry (LO) モード**を用意し、フロントエンドのパラメータをスイープして APE を詰める。
- 既存の既定パイプライン（RKO-LIO + graph_based_slam）を壊さずに、**追加の評価軸**として LO を整備する。

### 追加パイプライン（現状）
- **LO baseline**: `scanmatcher_node`（IMU 無効） + `graph_based_slam`  
  - launch: `lidarslam/launch/lo_slam.launch.py`
  - params: `lidarslam/param/lidarslam_lo.yaml`
  - bench: `scripts/run_lo_graph_benchmark.sh`
- **small_gicp LO**: `small_gicp_odom_node` + `graph_based_slam`  
  - launch: `lidarslam/launch/small_gicp_lo_slam.launch.py`
  - params: `lidarslam/param/small_gicp_kitti_velodyne.yaml`
  - bench: `scripts/run_small_gicp_graph_benchmark.sh`
  - sweep: `scripts/sweep_kitti_small_gicp.sh`

### ここまでの実行で見えた「ハマりどころ」（重要）
このセッションで、スクリプトは起動できたが **TUM が空（traj_raw.tum / traj_corrected.tum が 0 bytes）**になりやすい状況が確認できた。
主因は概ね次の 2 つ。

#### (A) rosbag2 play と購読 QoS の不一致（BestEffort vs Reliable）
- `small_gicp_odom_node` の購読は `rclcpp::SensorDataQoS()`（一般に **BestEffort**）。
- `ros2 bag play` の publisher 側が **Reliable** になり、接続が成立せず **コールバックが一度も来ない**ことがある。
- 目視の兆候:
  - `ros2 bag info` では `/kitti/velodyne/points` にメッセージが存在する
  - しかし `odom_to_tum` は “Subscribed ...” のログだけで、ファイルが増えない
  - `small_gicp_odom_node` 側ログは起動・終了しか残らない

**対策（実装済み）**
- `ros2 bag play --qos-profile-overrides-path <yaml>` を使い、`INPUT_CLOUD` の QoS を **best_effort + volatile** に固定する。
- `scripts/run_small_gicp_graph_benchmark.sh` / `scripts/run_lo_graph_benchmark.sh` が各出力ディレクトリに `rosbag2_play_qos.yaml` を生成して渡す。
- launch と logger の起動・初期化を待ってから bag 再生を始める。これで冒頭フレーム取り逃がしによる空 TUM を避ける。

#### (B) “graph の corrected が取れない” ＝ 失敗ではなく「短すぎる」場合がある
- 2 フレーム程度の短い bag だと `graph_based_slam` が `/modified_path` を出す前に終わることがある。
- その場合でも **フロントエンド odom（/small_gicp/odom）** が記録できていれば評価は可能。

**対策（実装済みの方針）**
- `traj_corrected.tum` が無い場合に `traj_raw.tum` をコピーして “corrected” として後段評価を続行する（スモーク用のフォールバック）。
- 本番スイープでは “corrected を必須” に戻すか、別指標として raw/corrected を分けて扱う。

#### (C) launch のパラメータ上書き順の罠
- `small_gicp_lo_slam.launch.py` の `parameters=[dict(overrides), param_file]` の順だと、**param_file が最後に勝って override が効かない**。
- override を効かせるには **param_file → overrides の順**にする必要がある。

**対策（実装済み）**
- `small_gicp_lo_slam.launch.py` の parameters の順序を調整し、CLI override が最優先で勝つようにした。

### スイープのやり方（引き継ぎ用・再現コマンド）
#### 最小（まず 00 だけ）
```bash
# KITTI のルートを指定（odometry dataset 構造）
bash scripts/sweep_kitti_small_gicp.sh --dataset /path/to/KITTI_odometry --sequences "00"
```

#### 複数（00/05/07）
```bash
bash scripts/sweep_kitti_small_gicp.sh --dataset /path/to/KITTI_odometry --sequences "00 05 07"
```

### 次にやること（TODO）
- `scripts/sweep_kitti_small_gicp.sh` の `CONFIGS` を広げる:
  - `ds`（downsampling_resolution）
  - `voxel`（IncrementalVoxelMap の voxel）
  - `corr`（max_correspondence_distance）
  - `range`（min/max）
  - `use_gicp`（ICP vs GICP）※GICP は共分散計算が重いので最後に
- ユーザー導線:
  - 公開推奨は引き続き **RKO-LIO + graph_based_slam**。
  - KITTI / Velodyne-only は評価・チューニング用の導線として README / docs から `download_kitti_odometry.sh` → `run_kitti_odometry_benchmark.sh --small-gicp` → `sweep_kitti_small_gicp.sh` に誘導する。
  - `datasets/`, `map.pcd`, `map_projector_info.yaml`, `pointcloud_map/` はローカルデータ/生成物として Git 管理外に置く。
- 成功条件:
  - `traj_raw.tum` が non-empty
  - `ape_raw_vs_gt.txt` が生成される（少なくとも raw 側）
  - スイープ後に `benchmark_summary.py` の md/csv が出る
