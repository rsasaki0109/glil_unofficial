# ICRA2025 Exact Sampling Notes

This fork tracks the main ideas from Koide et al., *Tightly Coupled Range
Inertial Odometry and Mapping with Exact Point Cloud Downsampling*, ICRA 2025.
The project page and paper describe a CPU-oriented SLAM pipeline that uses
registration error factors with exact point cloud downsampling.

Reference: [project page](https://staff.aist.go.jp/k.koide/projects/icra2025_es/)
and [paper](https://staff.aist.go.jp/k.koide/assets/pdf/icra2025.pdf).

## Core Idea

The exact sampling path builds a small weighted subset of residual rows that
preserves the quadratic linearization terms at a sampling pose:

```text
H = J^T W J
b = J^T W e
c = e^T W e
```

The practical benefit is that later factor linearization can evaluate far fewer
residual rows while keeping the same local quadratic model at the sampling point.
The paper also uses deferred sampling: do full residual evaluation first, then
extract and reuse a coreset once the linearization point is stable enough.

## Current Implementation

Implemented pieces in this fork:

- `glil::IntegratedVGICPCoresetFactor`
- `VGICP_CORESET` registration error factor type in odometry/global mapping
- Caratheodory-based row selection via the bundled `thirdparty/caratheodory2`
- deferred coreset reuse thresholds
- optional immutable coreset snapshots for deterministic debugging
- `IntegratedVGICPCoresetFactor::coreset_stats()` for programmatic inspection of
  selected residual rows, valid correspondences, weights, and extraction counts
- per-frame coreset performance counters in `[perftime]` logs
- CPU overlap estimation through `gtsam_points::FastOccupancyGrid` in the GLIL
  odometry path

Useful parameters:

| parameter | meaning |
|---|---|
| `registration_error_factor_type=VGICP_CORESET` | enables coreset VGICP factors |
| `coreset_target_size` | target number of selected residual rows |
| `coreset_num_clusters` | Fast-Caratheodory cluster count |
| `coreset_relinearize_thresh_trans` | translation drift threshold for re-extraction |
| `coreset_relinearize_thresh_rot` | rotation drift threshold for re-extraction |
| `coreset_method` | exact or diagnostic sampling mode |
| `occupancy_grid_resolution` | overlap grid resolution for keyframe decisions |

## Diagnostics

`IntegratedVGICPCoresetFactor::coreset_stats()` returns the current coreset
state after a factor has been linearized. The values are intentionally compact so
extensions and tests can check behavior without parsing logs:

- `source_points`, `valid_correspondences`, `selected_points`, and
  `selected_residual_rows` show how much the residual set was reduced.
- `weight_sum`, `target_size`, `num_clusters`, and `method` describe the selected
  coreset configuration.
- `correspondence_update_count` and `coreset_extraction_count` make deferred
  sampling behavior visible.

The CTest target `integrated_vgicp_coreset_factor` builds a small synthetic
Gaussian voxel map, linearizes a `VGICP_CORESET` factor, and checks that these
stats are populated. This gives CI a direct smoke test for the ICRA2025 exact
sampling path.

## Benchmark Command

Build the benchmark command and run it on a deterministic synthetic grid:

```bash
cmake --build build --target glil_coreset_benchmark
./build/glil_coreset_benchmark --repeat 8 --target 64 --clusters 64
```

The command compares three paths on the same fixed-target VGICP problem:

- `full_vgicp`: baseline evaluation over all valid residual rows.
- `uniform_sample`: diagnostic sampled residual rows with simple reweighting.
- `exact_caratheodory`: Fast-Caratheodory row selection through
  `IntegratedVGICPCoresetFactor`.

For recorded or exported point clouds, provide both target and source inputs:

```bash
./build/glil_coreset_benchmark \
  --target-cloud target_cloud_dir_or_file \
  --source-cloud source_cloud_dir_or_file \
  --voxel 0.5 \
  --target 64 \
  --clusters 64 \
  --repeat 4 \
  --csv
```

Supported real-cloud inputs are:

- GLIL/gtsam_points directories containing `points.bin` or `points_compact.bin`
- ASCII PCD files with `DATA ascii`
- whitespace, comma, or semicolon separated XYZ text files
- KITTI-style float32 XYZI `.bin` files
- compact float32 XYZ `.bin` files with `--target-format compact-bin` or
  `--source-format compact-bin`

Use `--initial-xyzrpy tx ty tz roll pitch yaw` when the source cloud needs a
non-identity initial pose relative to the target. Inputs without covariance
attributes receive an isotropic covariance from `--fallback-covariance`.

The default output reports first-call linearization time, repeated snapshot reuse
time, relative augmented-Hessian error against full VGICP, valid correspondences,
selected residual rows, selected source points, and coreset extraction count. Use
`--csv` when collecting results from multiple target sizes or datasets.

A healthy smoke run should show `full_vgicp` as the zero-error reference,
`uniform_sample` with non-zero relative augmented-Hessian error, and
`exact_caratheodory` close to machine precision with a single coreset extraction
when immutable snapshot reuse is enabled.

## Benchmark Summary Tool

`glil_coreset_benchmark_summary` reads one or more `--csv` outputs and renders a
baseline-relative comparison table. Pass `LABEL=path` when summarizing multiple
runs so each row is attributable in the resulting table:

```bash
glil_coreset_benchmark_summary \
  --csv sample=config/sample_coreset_benchmark.csv \
  --format markdown \
  --output coreset_benchmark_summary.md
```

Rendered against the bundled `config/sample_coreset_benchmark.csv` fixture the
summary looks like this:

| run | mode | selected rows | selected points | row ratio | point ratio | first us | reuse us | reuse speedup vs baseline | rel_aug_error |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| sample | `full_vgicp` | 2,048 | 2,048 | 1.0000 | 1.0000 | 14,200.0 | 14,150.0 | 1.00 | 0.000e+00 |
| sample | `uniform_sample` | 64 | 64 | 0.0312 | 0.0312 | 2,100.0 | 2,080.0 | 6.80 | 1.280e-01 |
| sample | `exact_caratheodory` | 64 | 52 | 0.0312 | 0.0254 | 3,400.0 | 210.0 | 67.38 | 4.800e-12 |

The `rel_aug_error` column is the relative augmented-Hessian error against the
baseline mode (`full_vgicp` by default). `reuse speedup vs baseline` divides the
baseline reuse time by each mode's reuse time, so values greater than `1.0` are
faster than the baseline. `row ratio` and `point ratio` show how aggressively
each mode reduced the residual and source set.

Use `--format csv` when a script needs to diff summary values across commits or
dataset configurations. Use `--strict` to fail a CI step when the baseline row
is missing or the exact mode exceeds `--exact-error-threshold` (default
`1e-3`).

## Reproduction Scope

The items below separate what directly tracks the ICRA 2025 paper from what is
a bridge/demo piece and what is open future work, so readers do not mistake the
repo for a full re-implementation of the paper's SLAM pipeline.

Directly tracked from the paper ("reproduction targets"):

- Fast-Caratheodory row selection for VGICP residuals
  (`glil::IntegratedVGICPCoresetFactor`, bundled `thirdparty/caratheodory2`).
- Augmented Hessian preservation as the correctness gate: the
  `exact_caratheodory` mode must reach the same `H`/`b`/`c` as `full_vgicp`
  up to floating-point precision (enforced by the benchmark summary tool's
  `--exact-error-threshold`).
- Deferred coreset reuse with immutable snapshots, including the
  `coreset_relinearize_thresh_*` thresholds that gate re-extraction.
- `VGICP_CORESET` as an opt-in registration error factor type in GLIM's
  odometry and global mapping paths through the GLIL fork.

Bridge / demo (inspired by the paper but not a 1:1 re-implementation):

- Synthetic Gaussian voxel benchmarks used by CTest and the sample fixture;
  they exercise the coreset path but are not the paper's datasets.
- Real-cloud benchmark inputs via `glil_coreset_benchmark --target-cloud ...
  --source-cloud ...`. These let users run the benchmark on their own data
  but the repo does not ship the exact point clouds used in the paper.
- `CloudLandmarkExtractor` and `glil_cloud_landmark_extractor` for
  geometry-based perception observation CSVs. Useful as a placeholder for
  a detector pipeline, but it is not a semantic detector and the paper does
  not rely on perception landmark factors.

Not implemented / explicit non-goals for this fork:

- A complete re-run of the paper's headline accuracy numbers on the paper's
  own datasets. This fork's reproduction scoreboard uses the MegaParticles
  bags, not the ICRA paper's evaluation set.
- Paper-table-level reporting (per-dataset coreset statistics, ablation
  tables). The benchmark summary tool is intentionally a single-run view.
- The paper's full mapping/loop-closure pipeline beyond what upstream GLIM
  already provides.

## Next Work

The remaining engineering targets are:

- a compact local occupancy bit-chunk grid fallback for environments where the
  `gtsam_points` helper is unavailable
- documentation of recommended settings for desktop CPU, Mini PC, and low-power
  embedded CPU runs
