# Benchmark Python install surface

The canonical benchmark/evidence implementation files remain in the
repository `scripts/` tree.  The `graph_based_slam` CMake install projects
those files into the collision-resistant `lidarslam_benchmark_tools` Python
package and installs the profile/configuration inputs under
`share/graph_based_slam/configs`.  There is no second source copy to edit.

Use the installed surface after sourcing the ROS/local setup files:

```bash
lidarslam_benchmark_tool evaluate_competitive_suite_gate --help
python3 -m lidarslam_benchmark_tools.evaluate_competitive_suite_gate --help
```

`lidarslam_benchmark_tool` accepts a simple module basename and dispatches it
through the installed package.  It refuses a repository checkout fallback.
The package also exposes `require_installed_surface()` for clean-install
smoke/claim tests.  Source-tree imports remain available for development and
tests, but they are not evidence of an installed release.

The runtime Python dependencies are declared by `graph_based_slam/package.xml`:
Python 3 and PyYAML.  ROS package dependencies remain in that package's normal
ament metadata.  A clean install must provide the package's own Python path;
tests must not add the repository root to `PYTHONPATH` to make imports pass.

The projection includes the dependency-light shared helpers under
`lidarslam_benchmark_tools.lidarslam_tools` and the Gaussian-splatting helpers
under `lidarslam_benchmark_tools.gaussian_splatting`.  Both are projected from
their one canonical source directory; no ambiguous top-level `scripts`,
`lidarslam_tools`, or Gaussian helper import is required.  Historical launcher
implementations resolve one another with `module_path()` from the installed
package rather than constructing a checkout path.

ROS1/ROS2 and rosbag2/TF imports are intentionally deferred until the command
that needs them runs.  Import-all checks therefore work on a host without
either ROS distribution, while an attempted ROS command fails with an
explicit dependency message.  This is an import-surface guarantee only; it
does not make a ROS runtime available.
