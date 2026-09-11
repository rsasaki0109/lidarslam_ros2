# lidarslam_ros2 Docs

<section class="hero">
  <div class="hero__copy">
    <div class="hero__eyebrow">ROS 2 LiDAR SLAM Docs</div>
    <h1>Pointcloud-map authoring, benchmark evidence, and browser-first map proof.</h1>
    <p>
      <code>lidarslam_ros2</code> is organized around a practical public path:
      build a pointcloud map, validate it, and open it through
      Autoware-compatible map workflows.
    </p>
    <div class="hero__badges">
      <span>RKO-LIO frontend</span>
      <span>graph_based_slam backend</span>
      <span>Foxglove proof path</span>
    </div>
    <div class="hero__actions">
      <a class="md-button md-button--primary" href="getting-started.html">Start Here</a>
      <a class="md-button" href="getting-started-ja.html">日本語で始める</a>
      <a class="md-button" href="autoware-map-authoring.html">Map Your Bag</a>
    </div>
  </div>
  <div class="hero__visual">
    <img src="assets/images/autoware_map_loader_proof.png" alt="Browser proof of an Autoware-compatible pointcloud map" />
  </div>
</section>

<section class="proof-grid">
  <article class="proof-card">
    <h2>Autoware-compatible proof</h2>
    <p>
      The public flow publishes a live <code>/map/pointcloud_map</code>,
      writes <code>map_projector_info.yaml</code>, and keeps map verification
      in the documented path.
    </p>
    <a href="autoware-foxglove.html">Open the Foxglove viewer path</a>
  </article>
  <article class="proof-card">
    <h2>Map cleanup with evidence</h2>
    <p>
      Save-time dynamic filtering reduces map size while preserving coarse
      footprint overlap. The validation reports track both reduction and tile
      overlap.
    </p>
    <img src="assets/images/dynamic_object_filter_bag6_summary.svg" alt="Dynamic-object filter benchmark summary" />
  </article>
</section>

## Start Here

<div class="card-grid">
  <a class="link-card" href="getting-started.html">
    <h3>Getting Started</h3>
    <p>Choose Docker, a local build, or your own bag without reading every workflow first.</p>
  </a>
  <a class="link-card" href="product-contract.html">
    <h3>Product Contract</h3>
    <p>Know the supported inputs, outputs, entrypoints, and explicit non-goals.</p>
  </a>
  <a class="link-card" href="v1-readiness.html">
    <h3>v1.0 Readiness</h3>
    <p>Run the fail-closed product audit and inspect every remaining evidence gate.</p>
  </a>
  <a class="link-card" href="distribution.html">
    <h3>Distribution</h3>
    <p>Install the CLI, understand supported platforms, and see the binary-release boundary.</p>
  </a>
  <a class="link-card" href="operational-reliability.html">
    <h3>Operational Reliability</h3>
    <p>Understand failure artifacts, termination handling, recovery, and open reliability gates.</p>
  </a>
  <a class="link-card" href="real-data-e2e.html">
    <h3>Pinned Real-data E2E</h3>
    <p>Inspect the nightly Jazzy gate from a fixed public MID-360 bag to a verified map bundle.</p>
  </a>
  <a class="link-card" href="autoware-map-authoring.html">
    <h3>Autoware-Compatible Map Authoring</h3>
    <p>The shortest product-level summary of the supported public path.</p>
  </a>
  <a class="link-card" href="autoware-quickstart.html">
    <h3>Advanced Autoware Compatibility</h3>
    <p>Use the older NTU VIRAL viewer and dogfood route after the product path.</p>
  </a>
  <a class="link-card" href="autoware-foxglove.html">
    <h3>Autoware Foxglove</h3>
    <p>Open the map loader output in a browser-first viewer path.</p>
  </a>
</div>

## Operations

<div class="card-grid">
  <a class="link-card" href="workflows.html">
    <h3>Operator Workflows</h3>
    <p>Required topics, optional GNSS, packet paths, and map-save flows.</p>
  </a>
  <a class="link-card" href="benchmarking.html">
    <h3>Benchmarking And Release Gate</h3>
    <p>Run the tracked benchmark suite and generate the published reports.</p>
  </a>
  <a class="link-card" href="comparison.html">
    <h3>Comparison</h3>
    <p>See the current public position and benchmark-backed configuration summary.</p>
  </a>
</div>

## Current Snapshot

| Area | Current public position |
| --- | --- |
| Main path | `RKO-LIO` + `graph_based_slam` |
| Public map output | `pointcloud_map/` + `map_projector_info.yaml` |
| Browser proof | Foxglove path documented and smoke-tested |
| Long-loop evidence | `MID360` |
| Ground-truth benchmark | `NTU VIRAL tnp_01` |
| Save-time cleanup | dynamic filter with cross-dataset validation |

## Releases

- [v0.9.1 release candidate](releases/v0.9.1.md)
- [v0.9.0 stable release](releases/v0.9.0.md)
- [v0.7.0 release candidate](releases/v0.7.0.md)
- [v0.6.0](releases/v0.6.0.md)
- [v0.5.0](releases/v0.5.0.md)
- [v0.3.0](releases/v0.3.0.md)
- [v0.2.2](releases/v0.2.2.md)
- [v0.2.1](releases/v0.2.1.md)
- [v0.2.0](releases/v0.2.0.md)

## Project

- [Product contract](product-contract.md)
- [v1.0 readiness audit](v1-readiness.md)
- [Operational reliability](operational-reliability.md)
- [Pinned real-data E2E gate](real-data-e2e.md)
- [Distribution and installed CLI](distribution.md)
- [v0.9 product roadmap](roadmap/v0.9.md)
- [Contributing](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/CONTRIBUTING.md)
- [Support](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/SUPPORT.md)
- [Security](https://github.com/rsasaki0109/lidar_slam_ros2/security/policy)
- [Governance](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/GOVERNANCE.md)

## Local Preview

Build the docs:

```bash
python3 -m mkdocs build --strict
```

Serve them locally:

```bash
python3 -m mkdocs serve
```
