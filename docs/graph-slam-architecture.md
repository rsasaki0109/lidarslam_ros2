# Graph SLAM architecture

The Graph SLAM package is being split into a thin ROS shell and a deterministic
mapping engine. The dependency direction is one way:

```text
ROS component / offline adapters
              |
              v
      GraphSlamApplication
              |
              v
     deterministic backend
              ^
              |
 clock, storage, and output ports
```

The backend must not include ROS APIs, read a clock, or access the filesystem.
Online and offline execution must enter through the same application API. ROS
messages, parameters, services, and executors belong at the outer boundary.

## Configuration and composition root

`GraphSlamConfig` is a source-private snapshot of the ROS parameters. Startup
has four explicit phases:

1. declare and load every parameter;
2. normalize compatibility values;
3. validate the complete snapshot;
4. compose the backend's domain-specific configuration objects.

The validated snapshot is immutable after construction. Adaptive edge weights
and other live values are separate runtime state. Pure builders in
`graph_slam_composition` translate the snapshot into descriptor, loop-search,
pose-graph, filtering, grid, and GNSS configuration. This keeps ROS parameter
names and defaults out of backend code and makes configuration mapping directly
unit-testable.

Filesystem setup, publishers, subscriptions, and services still occur in the
component during this first milestone. They are effects to move behind ports;
they are not configuration work.

## Migration milestones

### 1. Configuration and composition root

- one typed parameter snapshot and one startup validation path;
- pure domain configuration builders;
- immutable startup intent separated from runtime state;
- no hand-written backend DTO copying in the ROS component.

### 2. Unified application

`GraphSlamApplication` is now the ordered, ROS-free mapping workflow entry
point. Both the live component and deterministic bag replay submit the same
ordered submap prefixes through `processSubmaps()`. The application owns the
query cursor, stride scheduling, accepted/deduplicated loop-edge set, and the
pose-graph optimization command. This makes callback batching observationally
equivalent to submitting one submap at a time.

The adapters still convert ROS messages, load clouds, emit logs, and publish or
write results. Registration, voxel filtering, descriptor storage, and 3D-BBS
objects are injected into the application during this milestone so their
ownership can move as one coherent unit in milestone 3.

### 3. Backend engine ownership

`GraphSlamApplication` is now the aggregate lifetime boundary for the mapping
engine. It is constructed from plain configuration only. Its private engine
owns descriptor databases, registration, the shared voxel filter, 3D-BBS,
query scheduling, and the canonical deduplicated loop graph. Neither the ROS
component nor the offline runner can construct or retain those resources.

Descriptor aggregation and filtering also moved behind `processSubmaps()`.
Adapters now supply only ordered submap metadata and a synchronous raw-cloud
provider; batching therefore cannot select a different filtering path.
`GraphSlamStateSnapshot` and per-query events return graph state by value.
Pose-graph requests no longer accept loop edges from callers: optimization
uses the canonical graph induced by the supplied submap prefix, excluding any
later accepted edges after a batched drain.

Registration creation and invalid-method handling are identical online and
offline because they happen in the same constructor. The composition root
maps the validated ROS snapshot into the complete application configuration;
the offline adapter builds the same plain DTO from replay parameters.

### 4. External I/O ports

The outer boundary now exposes explicit clock, submap storage, diagnostics,
and map-output ports. The live ROS composition root binds those contracts to
the node clock, ROS logging, PCD storage, and filesystem artifact output. The
offline runner uses the same filesystem artifact adapter, while unit and
deterministic replay tests can bind manual-clock and in-memory adapters.

Pose-graph optimization no longer accepts a save path. It serializes the
canonical g2o graph to bytes in `OptimizationResult`; an outer map-output port
decides whether and where to persist those bytes. The same boundary owns
trajectory, loop-edge, bundle-manifest, diagnostic-report, grid metadata, and
PCD output. Consequently `graph_slam_application` neither links the
filesystem adapter nor observes a clock, logger, ROS API, or path.

Submap cache access is also a value-snapshot contract. In-memory and PCD
implementations return independent clouds, so callers cannot mutate storage
state through an alias. Port validation rejects partial composition, and unit
tests pin clock control, diagnostic ordering, snapshot isolation, exact byte
preservation, and append behavior.

### 5. Architecture hardening

The exported ROS component is now a 48-line registration and lifetime shell;
ROS subscriptions, message conversion, diagnostics, and publication live in a
separate outer adapter translation unit. A CI architecture contract caps the
component at 300 physical lines and verifies that it does not reach into the
mapping engine directly.

The same contract walks the complete repository-local include closure of
`GraphSlamApplication` and rejects ROS, rosbag, clock, stream-file, or
filesystem dependencies. It also inspects the CMake target boundary so the
filesystem adapter cannot be linked into the deterministic application.

Optimization and deterministic artifact rendering are one Application
transaction. A single canonical loop-graph snapshot produces the optimized
poses, g2o bytes, loop-edge CSV, and optimized TUM trajectory. Live callbacks
and offline replay both call `optimizeAndSerialize()`; their adapters only
choose output destinations. The parity test submits the same ordered input as
one live-style batch and as incremental replay, then requires byte-identical
loop CSV, optimized trajectory, and g2o artifacts.

Default CI, both supported ROS distributions, documentation checks, and the
release-readiness pass/fail guards remain required before the milestone PR is
made ready for review.

Each milestone is delivered as one reviewable pull request. Compatibility is
preserved at its boundary: existing parameter names, defaults, topics, and
services remain stable unless a migration is explicitly documented.
