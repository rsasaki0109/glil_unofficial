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

## Synthetic Benchmark

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

The default output reports first-call linearization time, repeated snapshot reuse
time, relative augmented-Hessian error against full VGICP, valid correspondences,
selected residual rows, selected source points, and coreset extraction count. Use
`--csv` when collecting results from multiple target sizes.

A healthy smoke run should show `full_vgicp` as the zero-error reference,
`uniform_sample` with non-zero relative augmented-Hessian error, and
`exact_caratheodory` close to machine precision with a single coreset extraction
when immutable snapshot reuse is enabled.

## Next Work

The remaining engineering targets are:

- a dataset-backed benchmark command that compares full VGICP, uniform sampling,
  and coreset sampling on recorded submap pairs
- a compact local occupancy bit-chunk grid fallback for environments where the
  `gtsam_points` helper is unavailable
- a stats-driven benchmark report for `VGICP_CORESET` parameters and debug counters
- documentation of recommended settings for desktop CPU, Mini PC, and low-power
  embedded CPU runs
