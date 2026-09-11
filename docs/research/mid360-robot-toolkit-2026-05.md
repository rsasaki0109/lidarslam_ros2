# MID-360 robot toolkit chain — 2026-05 アーカイブ

> 操作員向け session pipeline (PR #168-#177, inside-out 10 PR) の構築記録。
> 2026-06-07 の docs/research/project-plan-archive.md surgery（v0.4 roadmap §F）で `docs/research/project-plan-archive.md` §10 から抽出した
> 歴史的アーカイブ。foundation / analyzer / dashboard / record / readiness /
> public-RKO / session / bundle layer の詳細、ament lint の罠、重要ファイル、
> 3DGS QA candidate を保存する。live なステータスは `docs/research/project-plan-archive.md` §10 と
> memory `project_mid360_robot_toolkit_stack`。

### 10.1 何を作ったか

Jetson + Livox MID-360 を載せた robot で、現場の操作員が

  1. **bag を撮る → 録音を check → SLAM map を作る → public RKO ベースラインで設定の妥当性を gate → production-readiness を判定 → operator-facing dashboard + 配布可能 bundle を出す**

までを 1 コマンド (`run_mid360_robot_production_candidate_session.py --run …`) で通せる operator pipeline を、**10 PR (#168〜#177)** に inside-out で分割して develop に入れた。

ここでの「inside-out」は、**依存ツリーの leaf (foundation) → root (orchestrator) の順に narrow PR を積む**戦略のこと。9 PR 全部が `mid360_robot_tools` (PR #168) という self-contained foundation の上に乗っていて、各 PR は前段の PR でランドしたモジュールだけに依存する。

### 10.2 全 PR table

| Phase | PR | scripts/test 新規 | 役割 |
|-------|----|------|------|
| 1 | #168 | `mid360_robot_tools.py` (1423 行) + test | Foundation: RobotProfile / preflight builder / map-run planner / run-manifest writer / payload_to_json |
| 1 | #169 | `mid360_robot_loop_alignment_analyzer.py` (760 行) + `analyze_*.py` + test | PCD 由来の loop closure 候補に largest_component_ratio + local_cloud_checks |
| 1 | #170 | `mid360_robot_dashboard.py` (903 行) + `mid360_robot_production_candidate_bundle.py` (330 行) + CLI x2 + test x2 | Operator-facing HTML dashboard + tar.gz bundle (loop_alignment artifact が dashboard + bundle に統合され E2E 動作確認済) |
| 2 | #171 | `mid360_robot_record_tools.py` (220 行) + test | RobotProfile から `ros2 bag record` コマンドを構築 + 再現可能な manifest writer (json + md + profile snapshot) |
| 2 | #172 | `mid360_robot_production_readiness.py` (407 行) + `check_*.py` + test | 操作員向け production-readiness gate (bag_path / duration / topic rate / map verify / public RKO adoption gate を集約 PASS/FAIL + next_actions) |
| 2 | #173 | `mid360_robot_public_rko_quality_report.py` (838 行) + `mid360_robot_rko_config_adoption.py` (293 行) + CLI x2 + test x2 | sweep manifest → quality score + gate を計算、tracked config が gate-passing case と一致するか確認 |
| 2 | #174 | `mid360_robot_public_rko_sweep.py` (965 行) + `run_*.py` + test + configs/mid360_robot/rko_lio_mid360_*.yaml x2 | public MID-360 bag に対する RKO-LIO parameter sweep (per-case yaml override + timeout 付き subprocess 駆動 + runtime signature 解析) |
| 2 | #175 | `mid360_robot_public_rko_adoption_gate.py` (310 行) + `run_*.py` + test | sweep → quality → adoption を 1 entry に orchestrate (run / plan / from-existing mode) |
| 2 | #176 | `mid360_robot_production_candidate_session.py` (741 行) + `mid360_robot_production_candidate_bundle_import.py` (401 行) + CLI x3 (py + shell) + test x2 | **chain closing piece**: session orchestrator (recording → readiness → mapping → public gate → production gate → dashboard) + bundle import (tar.gz extract + verify + 再 gate) |
| 2 | #177 | `mid360_robot_recording_check_tools.py` (393 行) + `check_mid360_robot_recording.{sh,py}` + `record_mid360_robot_bag.sh` + `plan_mid360_robot_record.py` + `configs/mid360_robot/livox_mid360_default.yaml` + test | 録音後 check cascade (bag が record plan / robot profile に合っているか、topic 周波数 / frame id を確認)。PR #176 で skipif した record_only test がこれで自動 enable |

**Phase 1 (#168〜#170)**: 前セッションで先に landed (dashboard + bundle の loop_alignment 統合まで)。  
**Phase 2 (#171〜#177)**: 本セッションで連続 land。**7 PR / ~7400 行 / 全 5/5 CI green**。

### 10.3 依存ツリー

```
mid360_robot_tools (foundation, PR #168)
 ├── mid360_robot_record_tools (PR #171)
 │    └── mid360_robot_recording_check_tools (PR #177)
 ├── mid360_robot_loop_alignment_analyzer (PR #169)
 ├── mid360_robot_dashboard (PR #170)
 ├── mid360_robot_production_candidate_bundle (PR #170)
 ├── mid360_robot_production_readiness (PR #172)
 ├── mid360_robot_rko_config_adoption (PR #173)
 ├── mid360_robot_public_rko_quality_report (PR #173)
 ├── mid360_robot_public_rko_sweep (PR #174)
 │    └── mid360_robot_public_rko_adoption_gate (PR #175) — also depends on #173
 │         └── mid360_robot_production_candidate_session (PR #176) — depends on #170/171/172/175
 │              └── mid360_robot_production_candidate_bundle_import (PR #176)
 └── plan_mid360_robot_record (PR #177, uses #171)
```

各 PR は前段の PR の output (script + module API) だけに依存。CMakeLists.txt への `ament_add_pytest_test` の追加が PR 間で衝突 → develop merge ごとに rebase (10+ 回) して force-push-with-lease で揃えた。

### 10.4 ament lint の罠 (memory にも記録)

7 PR の連続 land 中に **3 種類の CI fail パターン**を踏んだ：

1. **`ament_copyright`**: `# Copyright 2026 Sasaki / # Software License Agreement (BSD 2-Clause Simplified License)` の 4 行 header だけだと `license=<unknown>` で fail。 `Redistribution and use ...` で始まる **BSD-2-Clause license body 全文 (約 23 行)** を入れる必要あり。テンプレは `test_aligned_trajectory_metrics.py` 先頭 29 行。
2. **`ament_flake8 I101` (Jazzy のみ)**: `from module import (A, B, C)` の中身は **case-insensitive アルファベット順**。`render_rko_quality_markdown` は `RKO_QUALITY_HTML` より前 (`r_e` < `r_k`)。lowercase が大文字より先に来る。
3. **`ament_flake8 I100`**: `from local_module import ...` を `import yaml` (third-party) より後に置くと "should be before 'import yaml'" で fail。**対策は `importlib.import_module('mid360_robot_*')` で lazy load する pattern**。test_mid360_robot_tools.py が既に使っている既存パターンを踏襲。

加えて **untracked dep**:

- `configs/mid360_robot/*.yaml` を PR に同梱しないと CI で `FileNotFoundError`。
- `scripts/run_*.sh` shell wrapper も untracked だと `subprocess.CalledProcessError: returncode 127`。

合計で **3 回の force-push rework + 1 個のテストを skipif で deferred** したが、最終的に 7 PR とも 5/5 green で landed。

### 10.5 残課題 (untracked)

- `check_jetson_mid360_host_readiness.py` + `jetson_mid360_host_tools.py` (Jetson 上の CPU / disk / cuda preflight) — 次の natural な PR
- `preflight_mid360_robot_bag.py`, `validate_mid360_robot_profile.py`, `rewrite_mid360_robot_bag_stamps.py` などの preflight 系
- `public_dataset` / `public_loop` / `sample_session` / `field_session` 系 (~15 scripts)
- 3DGS visual QA/export 系 (pointcloud_map + trajectory + loop candidates を operator が確認できる splat/HTML artifact にする)
- 関連 docs: `docs/jetson-mid360-robot-runbook.md`, `docs/jetson-mid360-robot-scope.md`, `docs/jetson-mid360-static-tf-worksheet.md`
- 実機 Jetson + MID-360 robot での dogfood 実走 (cloud distribution の dogfood-vs-bench discrepancy 調査込み)

### 10.6 重要ファイル (chain)

| ファイル | 説明 |
|---------|------|
| `scripts/mid360_robot_tools.py` | Foundation: RobotProfile / preflight / planner / payload_to_json |
| `scripts/mid360_robot_dashboard.py` | Operator HTML dashboard (loop_alignment 統合) |
| `scripts/mid360_robot_production_candidate_bundle.py` | tar.gz bundle 出力 |
| `scripts/mid360_robot_production_candidate_bundle_import.py` | tar.gz 受信 + verify + 再 gate |
| `scripts/mid360_robot_production_candidate_session.py` | Session orchestrator (chain closing piece) |
| `scripts/mid360_robot_production_readiness.py` | Production-readiness gate |
| `scripts/mid360_robot_recording_check_tools.py` | 録音後 check (bag ↔ record_plan ↔ profile) |
| `scripts/mid360_robot_public_rko_sweep.py` | public bag に対する RKO-LIO parameter sweep |
| `scripts/mid360_robot_public_rko_quality_report.py` | sweep manifest から quality gate report |
| `scripts/mid360_robot_public_rko_adoption_gate.py` | sweep → quality → adoption orchestrator |
| `scripts/mid360_robot_rko_config_adoption.py` | tracked RKO config と sweep best case の照合 |
| `scripts/mid360_robot_loop_alignment_analyzer.py` | PCD 由来 loop closure 候補の largest_component / local cloud check |
| `scripts/mid360_robot_public_segment_map_cloud_alignment.py` | reset済み start/end segment map をICPで剛体アラインし、loop drift をCloudAnalyzer gate化 |
| `scripts/mid360_robot_record_tools.py` | `ros2 bag record` コマンド + 再現可能 manifest |
| `scripts/run_mid360_robot_production_candidate_session.sh` | 操作員向け 1-コマンド エントリ |
| `scripts/import_mid360_robot_production_candidate_bundle.py` | 別マシンで bundle を受け取って recheck |
| `configs/mid360_robot/livox_mid360_default.yaml` | デフォルト robot profile (frames + expected topics) |
| `configs/mid360_robot/rko_lio_mid360_*.yaml` | sweep の base config (deskew off / low_voxel) |

### 10.7 3DGS visual QA/export candidate

3DGS (3D Gaussian Splatting) は入れる価値がある。ただし **SLAM の数値 gate
や production readiness の必須条件にはしない**。まずは operator / reviewer が
map の loop misalignment、split cloud、trajectory revisit を確認しやすくする
optional visual QA artifact として扱う。

#### 最終目標

**MID-360 で作った地図を、ブラウザで一発で見られる 3D map preview にする。**

成果物としては、RKO-LIO / graph_based_slam の `pointcloud_map/` から
`mid360_robot_3d_map_preview.html` を生成し、ブラウザで開くだけで map cloud、
trajectory、loop candidate marker を確認できる状態を目指す。これは 3DGS の
production training pipeline ではなく、3DGS 風の splat/point preview から始める。

#### 使いどころ

| Use case | 3DGS の役割 | core gate への扱い |
|---|---|---|
| loop alignment review | loop candidate 周辺を滑らかな splat scene として見せ、trajectory の往路/復路を重ねる | optional。PASS/FAIL は `mid360_robot_loop_alignment_analyzer.py` が持つ |
| map split diagnosis | connected components が分かれた場所を色分けして reviewer が見る | optional evidence |
| production candidate dashboard | `mid360_robot_session_dashboard.html` から 3D artifact へリンク | dashboard enhancement |
| bundle review | Jetson から持ち帰った bundle に軽量 3D preview を同梱 | bundle optional artifact |
| public demo | Autoware map verify PASS の map を人間に説明しやすくする | release/supporting material |

#### 重要な境界

- 現在の public MID-360 bags はカメラ画像を前提にしていない。したがって最初の
  3DGS は photorealistic radiance field ではなく、**LiDAR pointcloud 由来の
  geometry splat preview** として始める。
- synchronized camera images がある robot では、後で RGB 付き 3DGS training に
  拡張できる。しかし、現トラックでは camera calibration / image ingestion は
  production requirement に入れない。
- 外部 3DGS 実装を vendor しない。license / CUDA / PyTorch version / build time の
  リスクが大きいので、repo が最初に持つべき責務は **export manifest + lightweight
  viewer artifact + reproducible command**。
- 3DGS が綺麗でも、map verify / loop analyzer / production readiness が FAIL なら
  production candidate は FAIL のまま。3DGS は「説明」と「検査補助」であって、
  correctness proof ではない。

#### PoC design

最小 PoC は trainer ではなく exporter から始める。

| Phase | Artifact | 内容 | Test |
|---|---|---|---|
| A: splat export | `mid360_robot_3d_map_preview.json`, `mid360_robot_3d_map_preview.ply` | `pointcloud_map/` の PCD tiles を sample し、position / color を持つ preview PLY に変換。色はまず height-based | fixture PCD から deterministic PLY を生成 |
| B: loop overlay | `mid360_robot_3d_map_preview_overlay.json` | TUM trajectory、loop candidates、local cloud connected components を viewer overlay として出力 | fixture trajectory で candidate indices が JSON に残る |
| C: dashboard link | dashboard HTML | session dashboard から 3D preview artifact にリンク。bundle export/import でも optional artifact として保持 | dashboard test + bundle optional artifact test |
| D: viewer | `mid360_robot_3d_map_preview.html` | browser で map cloud + trajectory + loop marker を開ける軽量 viewer。重い dependency は optional | docs smoke + file existence |
| E: RGB 3DGS training | optional external command manifest | camera topics / calibration / images がある robot だけで trainer を呼ぶ。現段階では design only | no default CI |

#### Proposed module split

| Module | Responsibility |
|---|---|
| `scripts/mid360_robot_3d_map_preview.py` | PCD / trajectory / loop analyzer report を読み、HTML + PLY + overlay JSON を生成 |
| `scripts/export_mid360_robot_3d_map_preview.py` | CLI wrapper。`run_dir`, `--loop-alignment`, `--output-dir`, `--max-points` |
| `graph_based_slam/test/test_mid360_robot_3d_map_preview.py` | fixture binary/binary_compressed PCD と TUM から exporter を検証 |
| dashboard integration | `mid360_robot_dashboard.py` に optional 3DGS section |
| bundle integration | `mid360_robot_production_candidate_bundle.py` に optional include |

#### First implementation order

1. `mid360_robot_loop_alignment_analyzer.py` の PCD reader を再利用して PCD tiles を読む。
2. `max_points` と deterministic stride sampling で lightweight PLY を出す。
3. trajectory と loop candidates を overlay JSON に出す。
4. dashboard/bundle には artifact link だけ追加する。
5. public loop bag (`outdoor_kidnap_a + outdoor_kidnap_b`) の map run ができたら、loop
   analyzer report と 3DGS preview を並べて reviewer が確認する。

この順なら、3DGS を入れても SLAM core / Autoware map verify / production readiness
を汚さない。PoC が有用なら viewer と RGB training に進む。

#### Current public loop status (2026-05-25)

- `outdoor_kidnap_a + outdoor_kidnap_b` は raw sqlite merge 済み:
  `datasets/mid360_public_loops/outdoor_kidnap_raw/rosbag2`
  - `ros2 bag info`: 554.562s / 118,843 messages / 4.2GiB
  - topics: `/livox/points` PointCloud2 4,017, `/livox/imu` 110,612,
    `/livox/lidar` CustomMsg 4,214
  - split gap: 1.493804475s
- 実RKO-LIO投入:
  `output/mid360_public/outdoor_kidnap_ab_rko_tolerant`
  - `/map_save` 成功、Autoware map verify PASS
  - 3D map preview/dashboard 生成済み
  - ただし RKO trajectory は 203 poses / 28.4s / 43.1m で止まり、
    loop analyzer は FAIL (`nearest_revisit=22.120m`, loop candidates 0)
  - log: `Number of correspondences are 0` が 2,693 回、
    keypoint/drop 系 error が 3,814 回。`Received LiDAR scan ... delta` は 0
- bag側の実データ検証:
  - scan 203 から keypoint不足 zone が始まり、後段には再び有効scanがある。
    これは public `outdoor_kidnap` の kidnap/disconnected segment 性質で、
    旧continuous RKO-LIOでは post-kidnap を再捕捉できていなかった。
  - `scripts/analyze_mid360_robot_public_loop_cloud.py` は PASS。
    GT loop candidate 0 の実PointCloud2 overlap は
    median NN 0.250m / p90 0.548m / coverage within 1m 0.963。
    つまり public data の loop 自体は本物。
- continuous RKO-LIO kidnap relocalization:
  - RKO core に kidnap recovery path を追加。通常ICP失敗時に、pruneしない
    relocalization map へ coarse yaw search + ICP で再捕捉する。
    keypoint不足scanは時刻を進めてdropし、relocalization失敗時のみlocal resetへ
    fallbackする。
  - tracked config:
    `configs/mid360_robot/rko_lio_mid360_kidnap_tolerant.yaml` は
    `enable_kidnap_relocalization: true`,
    `reset_on_registration_failure: true`,
    `max_scan_delta_sec: 10000.0`。
  - 旧continuous gateは generic loop candidate だけを見ていたため浅かった。
    旧runは nearest revisit 0.162m でも public GT start/end endpoint が
    153.202m ずれていたので、completion 判定から外した。
  - 実public merged bag final run:
    `output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final`
    は RKO offline completion。RKO trajectory 2896 poses / 553.801s /
    path 882.955m。invalid scan drop 1121、global relocalization event 1。
  - `/map_save` 成功、Autoware verify PASS
    (`verify_autoware_map.log`: 8 PASS / 1 WARN / 0 FAIL)。
  - loop alignment analyzer は PASS:
    `output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final/mid360_robot_loop_alignment.json`
    で loop candidates 20、nearest revisit 0.162m、max loop distance 0.180m。
  - continuous completion gate を追加:
    `scripts/run_mid360_robot_public_continuous_relocalization_gate.py`。
    実artifact
    `output/mid360_public/continuous_relocalization_gate/mid360_robot_public_continuous_relocalization_gate.json`
    は PASS。public endpoint は GT start stamp 1693922461.499998 と
    GT end stamp 1693922994.700686 の最近傍poseで 2.515m (threshold 5.000m)。
    checks:
    continuous RKO trajectory complete, Autoware map verify PASS,
    loop alignment PASS, public loop endpoint relocalized, kidnap relocalization
    event present, offline node completed, tracked kidnap config matches run
    config。
  - 3D map preview/dashboard も同runで生成済み:
    `mid360_robot_3d_map_preview.html`,
    `mid360_robot_session_dashboard.html`。
- gate修正:
  - RKO quality/adoption gate に `trajectory_duration` を追加。
    map verify PASS でも trajectory が短すぎる case は gate FAIL になる。
  - `rko_sweep_loop_outdoor_kidnap_tolerant_v3` は map verify PASS だが
    trajectory 28.70s / keypoint drop 1,115 のため quality status `WARN`,
    gate pass 0。これで浅い production PASS を防げる。
- segment reset 実行状況:
  - `scripts/plan_mid360_robot_public_loop_segment_reset.py` は PASS。
    GT loop start は `segment_000`, loop end は `segment_012` に対応。
  - `segment_000` / `segment_012` はそれぞれ単体RKO-LIOで `/map_save` 成功、
    Autoware map verify PASS。RKO offline pose は 203 / 220。
  - `scripts/analyze_mid360_robot_public_segment_map_cloud_alignment.py` を追加。
    reset後の start/end segment map をICP剛体アラインし、median/p90/coverageで
    loop drift をgateできる。
  - 実データalignment gate:
    `output/mid360_public/outdoor_kidnap_segment_reset_alignment/mid360_robot_public_segment_map_cloud_alignment.json`
    は PASS。crop radius 20m、start/end analysis points 4,525 / 7,291、
    aligned median NN 0.632m、p90 2.107m、coverage within 1m 0.690。
  - dashboard は `mid360_robot_public_segment_map_cloud_alignment.json` を読み、
    `Segment Map Cloud Alignment` panel と check table に表示できる。
  - production candidate bundle は segment map alignment JSON/Markdown/PLY を
    optional artifact として同梱できる。requiredにはしないので、未生成でも
    bundle verifyは落とさない。
  - production candidate session CLI に `--segment-map-alignment <json>` を追加。
    外部alignment reportを渡すと session `artifact_paths` にJSON/Markdown/PLYが入り、
    dashboard表示、bundle export、bundle import/recheck後dashboardまで伝播する。
  - public completion gate を追加:
    `output/mid360_public/completion_gate/mid360_robot_public_completion_gate.json`
    および `.md` は PASS。`completion_ready=true`,
    scope は `public_mid360_segment_reset_loop_completion`。
    11/11 checks PASS:
    public loop cloud, segment reset plan, start/end segment RKO completion,
    start/end Autoware map verify, segment map alignment, RKO adoption gate,
    tracked config == top gate-pass config, dashboard presence,
    production candidate entrypoints presence。
    `run_release_readiness_checks.sh --public-mid360-completion` からも
    hard gate として呼べるように接続済み。
    これは「public MID-360 real-data で segment-reset loop path が完成」の判定。
    continuous RKO-LIO の完成判定は上記 continuous relocalization gate が担当する。
