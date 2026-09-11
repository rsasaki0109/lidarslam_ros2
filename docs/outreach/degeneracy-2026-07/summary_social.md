# Social Copy: LiDAR degeneracy resilience (fog / tunnel A/B, 2026-07)

Suggested attachment (pick one per post):

- `figures/fog_trajectory_xy.png`
- `figures/tunnel_trajectory_xy.png`
- `figures/gate_coverage_concept.png`

Link: <https://github.com/rsasaki0109/lidar_slam_ros2>
Article: `article_ja.md` (this directory)

## Japanese

### 案A — 数値訴求(添付: fog_trajectory_xy.png または tunnel_trajectory_xy.png)

濃霧の廊下・トンネルでLiDAR SLAMが退化する問題を検証。Hessian固有値ゲートは霧中で
一度も発火せず(クラッタロック)。radar速度不一致ゲートに切替え、fog 32.80→22.68m
(-31%)、tunnel到達98.7→495.3m(真値約500mの99%)まで改善。(144字)

### 案B — 教訓訴求(添付: gate_coverage_concept.png)

LiDAR SLAMの退化補正が効くかは『ゲートのカバレッジ』で決まるという話。固有値ゲートは
硬い退化しか拾えず、霧のクラッタロックは一度も検出できず。radar/intensityとICPの
速度不一致を見るゲートに切替えたら fog -31%、tunnel到達距離は約4.6倍に。(141字)

### 案C — radarなし環境訴求(添付: tunnel_trajectory_xy.png)

radar非搭載でも部分回復できないか試した記録。intensityの速度不一致ゲート(Hessian
不要)でトンネル到達距離が98.7→153.75mに改善。ただしfogでは逆に悪化(エアロゾルの
相関はノイズを拾う)— センサ特性で住み分けが必要という結果です。(132字)

### 案D — 正直な失敗訴求(添付: gate_coverage_concept.png)

『Hessian固有値で弱い方向を検出してprior注入』は王道だけど、霧のクラッタロックにも
トンネルのintensity拘束にもほぼ効きませんでした。効いたのはradar/intensityとICPの
速度不一致を見るゲート。効かなかった実験も含めて公開してます。(132字)

## English

### Draft — headline numbers (attach: fog_trajectory_xy.png or tunnel_trajectory_xy.png)

We benchmarked LiDAR SLAM degeneracy recovery on fog and tunnel corridors.
Hessian eigenvalue gates never fired during fog's clutter-lock. A radar-vs-ICP
velocity disagreement gate cut fog drift 32.80->22.68m (-31%) and tunnel reach
98.7->495.3m (99% of ~500m truth). (267 chars)

## Alt Text (for figure attachments)

- `fog_trajectory_xy.png`: Top-down XY trajectory plot comparing LiDAR SLAM
  baseline vs. a radar-disagreement-gated run on a fogged corridor, showing
  the baseline drifting 32.80 m from the loop start point vs. 22.68 m for the
  gated correction.
- `tunnel_trajectory_xy.png`: Top-down XY trajectory plot comparing baseline,
  radar-corrected (scale 1.05), and intensity-only corrected LiDAR SLAM runs
  in a ~500 m tunnel, showing how far each configuration tracks before losing
  the corridor.
- `gate_coverage_concept.png`: Timeline bar chart contrasting a Hessian
  eigenvalue degeneracy gate (fires on 85 of 1723 scans, all in the final 10
  seconds of a fog recording) against a sensor-disagreement gate (849 scans
  corrected across the foggy, clutter-locked stretch).
