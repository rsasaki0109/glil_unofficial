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

To also create a runnable config root in the same pass, add `--config-root`:

```bash
glil_cloud_landmark_extractor \
  --batch-csv frames.csv \
  --base-dir ./scans \
  --format kitti-bin \
  --voxel 1.0 \
  --min-points 8 \
  --max-landmarks 256 \
  --output cloud_landmarks.csv \
  --config-root run_config \
  --time-tolerance 0.10
```

`--path-column`, `--stamp-column`, and `--pose-columns` adapt other trajectory
CSV schemas without rewriting the file. `--skip-invalid-rows` keeps long dataset
conversion jobs running when a row or cloud file is bad.

To make an existing perception CSV runnable by the GLIL global mapping injector,
generate a small config root:

```bash
glil_perception_config_generator \
  --config-root run_config \
  --csv cloud_landmarks.csv \
  --time-tolerance 0.10 \
  --allowed-class-ids cloud_landmark
```

Both the extractor `--config-root` path and the standalone generator write or
update `config_perception.json`, add `libperception_csv_injector.so` to
`config_ros.json`, and link both files from `config.json`. Existing JSON files
are parsed with comment support, but rewritten as plain formatted JSON.

## Perception Workflow

Use `glil_perception_workflow` when preparing a dataset run. It keeps the
landmark CSV, runnable config root, and readiness report together so the
perception side of an experiment can be archived or attached to a reproduction
issue.

For point-cloud batches, the workflow runs extraction, config generation, and
reporting in one pass:

```bash
glil_perception_workflow \
  --batch-csv frames.csv \
  --base-dir ./scans \
  --cloud-format kitti-bin \
  --run-dir perception_run \
  --voxel 1.0 \
  --min-points 8 \
  --max-landmarks 256 \
  --time-tolerance 0.10
```

This creates `perception_run/cloud_landmarks.csv`,
`perception_run/config/config_perception.json`, and
`perception_run/perception_report.md`. The generated config references the CSV
relative to `perception_run/config`, so the run directory can be archived as one
bundle. Add `--submap-stamps submaps.csv` to measure how many observations can
actually match GLIL submap timestamps.

For an existing detector CSV, skip extraction and generate only the runnable
config root and report:

```bash
glil_perception_workflow \
  --observations-csv detector_landmarks.csv \
  --run-dir perception_run \
  --allowed-class-ids pole,sign,reflector \
  --rejected-class-ids car,person,bus \
  --report-format csv
```

Use `--tool-dir build` when the tools are built locally but not installed. The
script also accepts explicit `--extractor`, `--config-generator`, and
`--report-tool` paths for CI jobs or containerized pipelines.

## Perception Factor Report

Use `glil_perception_factor_report` before a long mapping run to check whether a
CSV is likely to produce useful factors under the same filters as the injector:

```bash
glil_perception_factor_report \
  --csv cloud_landmarks.csv \
  --config-root run_config \
  --submap-stamps submaps.csv \
  --format markdown \
  --output perception_report.md
```

The report summarizes observation count, accepted/rejected counts, class
breakdown, unique landmarks, confidence and covariance statistics, robust-loss
settings, and timestamp match rates against optional submap stamps. The
`--submap-stamps` file can be a one-column timestamp list or a CSV with a
`stamp` column. Use `--format csv` for scripts.

## Perception Report Summary

Use `glil_perception_report_summary` when comparing several perception-enabled
runs or when preparing a reproduction issue. It reads CSV reports generated by
`glil_perception_factor_report --format csv` and emits one compact readiness
table:

```bash
glil_perception_report_summary \
  --report baseline=baseline/perception_report.csv \
  --report tuned=tuned/perception_report.csv \
  --format markdown \
  --output perception_summary.md
```

The summary status is `PASS` when `injectable=yes`, the accepted observation
count meets `--min-accepted`, and `accepted_match_rate` meets
`--min-accepted-match-rate` when timestamp matching metrics are present. Use
`--format csv` to join perception readiness with registration or APE tables.

## Perception Run Comparison

`glil_perception_run_compare` takes one or more run directories and produces a
single Markdown or CSV table that places a perception-enabled run alongside a
baseline. It reads `evo_ape.txt` for trajectory metrics and the optional
`perception_report.csv` for injectable status, accepted observations, match
rate, and rejection reasons:

```bash
glil_perception_run_compare \
  --run baseline=run_baseline \
  --run perception=run_perception \
  --baseline-label baseline \
  --format markdown \
  --output perception_on_off.md
```

Run with the bundled fixture the output looks like this:

| run | perception | status | RMSE (m) | ΔRMSE vs baseline | accepted obs | accepted match rate | rejected class | rejected low conf | unique landmarks |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | off | PASS | 1.080450 | 0.000000 | NA | NA | NA | NA | NA |
| perception | on | PASS | 1.018235 | -0.062215 | 4 | 100.000000% | 1 | 0 | 3 |

The table is deliberately conservative. It does not claim perception factors
improve RMSE; it records observation counts, accepted factors, timestamp match
rates, and rejection reasons so that perception-enabled runs can be compared
honestly against a baseline. The `perception` column is `on` when the run's
report has `injectable=yes`, `loaded` when a report exists but injection was
disabled, and `off` when no perception report is present in the run directory.

Use `--strict` to fail a CI step when any run directory is missing its APE
file. Use `--format csv` to pipe the rows into an external spreadsheet or
additional processing.

### Real-run example: indoor_easy_01

Running the MegaParticles `indoor_easy_01` bag twice under the same binary
and comparing the two run directories produces the following table:

| run | perception | status | RMSE (m) | ΔRMSE vs baseline | accepted obs | accepted match rate | rejected class | rejected low conf | unique landmarks |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | off | PASS | 1.019250 | 0.000000 | NA | NA | NA | NA | NA |
| perception | on | PASS | 1.019281 | +0.000031 | 8 | 100.000000% | 0 | 0 | 3 |

The `perception` run used the baseline config with
`libperception_csv_injector.so` loaded and a `config_perception.json` pointing
at a CSV of eight synthetic observations (three `pole`, three `reflector`, two
`sign`) whose timestamps were taken from the baseline's LiDAR stream.

Two honest observations from this run:

- The ΔRMSE of `+31 µm` is below the mapping pipeline's numerical noise floor.
  The perception factors did not meaningfully change the trajectory in this
  experiment, and this table is not evidence that perception factors improve
  accuracy on their own.
- `accepted obs = 8` in the table comes from the `glil_perception_factor_report`
  readiness check, which confirms the CSV and config would accept all eight
  observations. The runtime injection log for the same run reports
  `poses=1 matched=1 accepted=1 inserted_landmarks=1`: only one submap stamp
  aligned within `time_tolerance=2.0s` with `consume_once=true`, so seven of
  the eight observations did not attach to a factor. Runtime-injected factor
  counts depend on how well the observation stamps align with submap
  boundaries, which the readiness report cannot predict.

The value of this run is end-to-end pipeline validation: the CSV loader, the
injector extension module, and the global-mapping factor builder all produced
the expected log line (`perception CSV injection poses=... matched=...
accepted=...`) and left the rest of the GLIL state consistent with the
baseline. A perception experiment that aims to influence RMSE will need a
detector that produces observations aligned with submap boundaries and enough
matched factors to move the optimum beyond the mapping noise floor.

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
- `glil_perception_config_generator` and extractor `--config-root` wiring for
  turning a perception CSV into a runnable config root
- `glil_perception_workflow` for one-command point-cloud or existing-CSV
  preparation of the landmark CSV, runnable config root, and readiness report
- `glil_perception_factor_report` for reporting perception CSV factor readiness,
  class filtering, confidence, covariance, and timestamp match rates
- `glil_perception_report_summary` for comparing readiness reports across runs
  and exporting reproduction-friendly markdown or CSV tables
- `glil_perception_run_compare` for placing a perception-enabled run next to a
  baseline run and rendering RMSE, accepted factors, match rate, and rejection
  reasons in a single Markdown or CSV table
- `libperception_csv_injector.so` for optional global-mapping CSV factor
  injection
- `config/sample_perception_observations.csv` and
  `config/sample_perception_factor_report.csv` for CSV injector and summary
  smoke tests
- CTest smoke tests that check residuals, Jacobians, confidence-weighted noise,
  CSV parsing, cloud landmark extraction, robust graph insertion, landmark
  optimization, report generation, and existing-CSV workflow wiring

Not implemented yet:

- ROS/ROS 2 perception message adapters
- detector-stream buffering beyond offline CSV timestamp association
- long-lived landmark lifecycle policy, e.g. pruning, merging, and relabeling
