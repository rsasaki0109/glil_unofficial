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

## Next Work

The remaining engineering targets are:

- a dedicated benchmark command that compares full VGICP, uniform sampling, and
  coreset sampling on the same submap pairs
- a compact local occupancy bit-chunk grid fallback for environments where the
  `gtsam_points` helper is unavailable
- CI smoke coverage for `VGICP_CORESET` parameters and debug counters
- documentation of recommended settings for desktop CPU, Mini PC, and low-power
  embedded CPU runs
