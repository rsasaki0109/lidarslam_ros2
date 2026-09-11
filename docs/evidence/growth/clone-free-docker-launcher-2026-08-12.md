# Clone-free Docker launcher — 2026-08-12

> Decision: **LOCAL_PIPELINE_READY / ASSET_NOT_PUBLISHED**
>
> Remote mutations performed: **none**

## Product gap

GLIM's strongest installation advantage is its prebuilt PPA route for Ubuntu
22.04/24.04 on amd64 and arm64. Its official Docker route is also prebuilt,
but asks the operator to clone configuration files and assemble the container
command. lidar_slam_ros2 already offered a safer one-command own-bag Docker
route, but the launcher itself still had to come from a repository checkout.

This increment removes that final source-tree dependency from the delivery
design. The same single Bash file can now be attached directly to a stable
release as `lidarslam-map-docker`. The repository form remains runnable and
explicitly identifies itself as `development (working-tree)`.

Official comparison sources:

- [GLIM installation](https://koide3.github.io/glim/installation.html)
- [GLIM Docker images](https://koide3.github.io/glim/docker.html)
- [GLIM getting started](https://koide3.github.io/glim/quickstart.html)

## Release identity and fail-closed behavior

`scripts/build_docker_launcher_asset.py` accepts one semantic release tag and
one exact 40-character lowercase Git revision. It requires unique development
markers, validates the rendered script with `bash -n`, creates the output with
exclusive semantics, and makes it executable. It does not use the network or
replace an existing path.

The generated launcher:

- reports its embedded release tag and source revision with `--version`;
- defaults to the matching `v<VERSION>-humble` or `v<VERSION>-jazzy` image;
- validates the bag and output before contacting Docker;
- mounts the bag read-only and disables container networking;
- resolves the selected image to an immutable local image ID;
- checks the high-level `lidarslam-map start` contract before output creation;
- refuses overlapping, reused, symlinked, or broad output paths; and
- claims completion only when the session page, map manifest, and validation
  receipt all exist as regular files.

The release workflow builds and shell-validates this asset from the exact tag
checkout, uploads it with the deterministic release bundle, attests both, and
attaches it to the GitHub Release. The read-only published-release audit does
not execute downloaded shell code. It checks bounded UTF-8 bytes, exact
version/revision markers, required isolation and result fragments, and exact
asset inventory.

## Historical compatibility

The published v0.9.0 release must remain reproducibly auditable with its six
original recovery assets. `check_published_release.py` therefore keeps the
six-asset contract for v0.9.0 and requires `lidarslam-map-docker` as a seventh
asset beginning with v0.9.1. An unexpected launcher on the historical release,
a missing launcher on the next release, or a launcher bound to another commit
all fail closed.

## Local verification

- Docker launcher and deterministic builder: 21 focused tests passed;
- historical and next-release publication contracts: 37 combined focused
  release/launcher tests passed;
- generated v0.9.1 test asset was executable, reported the expected tag and
  revision, and selected `v0.9.1-humble` in a Docker-free dry run;
- malformed tags/revisions, marker drift, symlink sources, repeated output,
  missing assets, unexpected historical assets, and revision tampering were
  rejected;
- Python style, docstrings, copyright, shell syntax, and JSON parsing passed
  for the initial implementation.
- the modified live audit still reports the historical public v0.9.0 release
  as `PUBLISHED`, with all six original asset contracts and both live GHCR
  version-tag digests passing.
- the complete maintained product gate passes 2,098 tests
  (`graph_based_slam` 1,428 and `lidarslam` 670), with 13 known skips and no
  failures.

No release asset was generated with a false candidate commit. A real asset can
exist only after a reviewed exact tip is committed, tagged, and processed by
the authorized release workflow. The remaining proof is a published candidate
asset followed by a timed clean-machine own-bag trial from the documented
download command.
