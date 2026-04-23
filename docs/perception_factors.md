# Perception Factors

GLIL now includes a small perception-factor foundation for adding external
object or landmark observations to the factor graph without linking a neural
network or detector into the core library.

## Model

`glil::PerceptionLandmarkFactor` constrains a pose and a 3D landmark:

```text
error = pose.transformTo(landmark_world) - measured_position_sensor
```

The measurement is a landmark position in the sensor/body frame. The landmark is
stored as a `gtsam::Point3` value in the world frame. This is intentionally the
same geometric shape used by common object, pole, reflector, sign, and fiducial
frontends after they have produced a stable track ID.

## Observation Schema

Use `glil::PerceptionObservation` as the frontend-neutral container:

| field | meaning |
|---|---|
| `stamp` | sensor timestamp in seconds |
| `class_id` | semantic label such as `pole`, `sign`, `cone`, or `fiducial` |
| `landmark_id` | stable track/landmark ID from the perception frontend |
| `position_sensor` | 3D landmark measurement in the sensor/body frame |
| `covariance` | 3x3 measurement covariance |
| `confidence` | frontend confidence in `[0, 1]` |

`make_perception_noise_model()` converts covariance and confidence into a
GTSAM diagonal noise model. Lower confidence increases sigma by
`1 / sqrt(confidence)`.

## CSV Input

Offline detectors can feed observations with a compact CSV file. The diagonal
covariance schema is:

```csv
stamp,class_id,landmark_id,x,y,z,cov_xx,cov_yy,cov_zz,confidence
12.3,pole,42,4.0,0.2,1.1,0.04,0.04,0.09,0.8
```

For detectors that estimate cross-axis covariance, the loader also accepts a full
3x3 covariance layout:

```csv
stamp,class_id,landmark_id,x,y,z,cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz,confidence
```

Use `load_perception_observations_csv()` for files or streams. Comment lines
starting with `#` and a header row are skipped.

## Point Cloud Landmark Extraction

`glil_cloud_landmark_extractor` converts a raw point cloud into the same CSV
schema. It groups finite points into world-frame voxels, emits each dense voxel
as one `cloud_landmark` observation, and assigns deterministic landmark IDs from
the voxel coordinate. Passing the sensor pose with `--pose-xyzrpy` makes those
IDs stable across frames that see the same world voxel.

```bash
glil_cloud_landmark_extractor \
  --input scan.bin \
  --format kitti-bin \
  --stamp 12.3 \
  --pose-xyzrpy 0 0 0 0 0 0 \
  --voxel 1.0 \
  --min-points 8 \
  --max-landmarks 256 \
  --output cloud_landmarks.csv
```

Supported input formats are GLIL/gtsam_points directories, ASCII PCD, XYZ text,
KITTI float32 XYZI `.bin`, and compact float32 XYZ `.bin`. The extractor is not
a semantic detector; it is a lightweight geometric frontend for smoke tests,
repeatable ablations, and datasets where stable poles, signs, reflectors, or
fiducial-like clusters can be isolated by range/voxel settings. Set `--class-id`
when the output should pass a stricter `allowed_class_ids` filter.

Use `--full-covariance` to write all nine covariance terms. Without it, the tool
writes the compact diagonal covariance CSV accepted by the default loader.

For multi-scan extraction, pass a batch CSV. The default header is:

```csv
path,stamp,tx,ty,tz,roll,pitch,yaw
scan_000000.bin,12.30,0,0,0,0,0,0
scan_000001.bin,12.40,0.1,0,0,0,0,0.01
```

Then run:

```bash
glil_cloud_landmark_extractor \
  --batch-csv frames.csv \
  --base-dir ./scans \
  --format kitti-bin \
  --voxel 1.0 \
  --min-points 8 \
  --max-landmarks 256 \
  --output cloud_landmarks.csv
```

`--path-column`, `--stamp-column`, and `--pose-columns` adapt other trajectory
CSV schemas without rewriting the file. `--skip-invalid-rows` keeps long dataset
conversion jobs running when a row or cloud file is bad.

## Global Mapping CSV Injector

`libperception_csv_injector.so` is an optional extension module that loads the
CSV format above and injects `PerceptionLandmarkFactor`s when a new global
submap pose is inserted. It is disabled by default, so existing GLIL runs do not
change unless the module is explicitly loaded.

To enable it:

1. Add `libperception_csv_injector.so` to `glil_ros.extension_modules` in
   `config_ros.json`.
2. Set `perception_csv_injector.enabled` to `true` in `config_perception.json`.
3. Set `perception_csv_injector.csv_path` to a CSV file. Relative paths are
   resolved from `global.config_path`.

The injector associates observations to the pending `X(submap_id)` pose by
matching observation `stamp` against the submap origin frame stamp within
`time_tolerance` seconds. With `consume_once=true`, each matched observation is
used for the first associated submap only.

Default class filters reject common dynamic objects such as `car`, `person`, and
`bus`. For stable landmarks, set `allowed_class_ids` to labels such as `pole`,
`sign`, `reflector`, or `fiducial`, and keep `initialize_missing_landmarks=true`
when the CSV carries stable landmark IDs.

For a smoke test, point `csv_path` at `sample_perception_observations.csv`, set
`allowed_class_ids` to `pole`, `sign`, and `reflector`, and add
`libperception_csv_injector.so` to `config_ros.json`. The sample includes a
`car` row so the class rejection path is exercised.

## Robust Loss

Perception frontends usually produce occasional outliers. Set `robust_loss` to
`HUBER`, `CAUCHY`, or `TUKEY` to wrap each perception factor noise model in a
GTSAM robust noise model. `robust_loss_width` is interpreted in whitened units,
so a Huber width of `1.5` means the quadratic-to-linear transition happens at
roughly 1.5 sigmas after covariance and confidence scaling. `NONE`, `OFF`, `L2`,
or a non-positive width disables robust wrapping.

## C++ Example

```cpp
#include <glil/factors/perception_landmark_factor.hpp>
#include <glil/perception/perception_factor_builder.hpp>

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

glil::PerceptionObservation obs;
obs.stamp = 12.3;
obs.class_id = "pole";
obs.landmark_id = 42;
obs.position_sensor = gtsam::Point3(4.0, 0.2, 1.1);
obs.covariance = gtsam::Matrix3::Identity() * 0.04;
obs.confidence = 0.8;

glil::PerceptionFactorBuilderParams params;
params.allowed_class_ids = {"pole", "sign", "fiducial"};
params.min_confidence = 0.5;
params.robust_loss = "HUBER";
params.robust_loss_width = 1.5;

glil::PerceptionFactorBuilder builder(params);
builder.add_observation(graph, values, X(keyframe_id), current_pose, obs, &current_estimate);
```

## Integration Point

The recommended first integration point is a module that listens to perception
frontend outputs and injects factors from `GlobalMappingCallbacks::on_smoother_update`.
That keeps detector dependencies outside the core and lets the mapping backend
consume only timestamped geometric observations.

Odometry-level injection is possible through
`OdometryEstimationCallbacks::on_smoother_update`, but global mapping is safer
for the first implementation because landmarks can persist across submaps and
loop closures.

## Current Scope

Implemented now:

- `PerceptionObservation`
- `make_perception_noise_model()`
- `PerceptionLandmarkFactor`
- CSV/stream observation loading
- `PerceptionFactorBuilder` for class filtering, robust loss wrapping, landmark
  initialization, and factor insertion
- `CloudLandmarkExtractor` and `glil_cloud_landmark_extractor` for converting
  real point clouds into perception-observation CSVs
- `libperception_csv_injector.so` for optional global-mapping CSV factor
  injection
- `config/sample_perception_observations.csv` for CSV injector smoke tests
- a CTest smoke test that checks residuals, Jacobians, confidence-weighted noise,
  CSV parsing, cloud landmark extraction, robust graph insertion, and landmark
  optimization

Not implemented yet:

- ROS/ROS 2 perception message adapters
- detector-stream buffering beyond offline CSV timestamp association
- long-lived landmark lifecycle policy, e.g. pruning, merging, and relabeling
