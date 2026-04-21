# Fair Comparison Report: CPU VGICP vs GLIL VGICP_CORESET

Updated: 2026-04-15

## Dataset

- Sequence: `outdoor_hard_01a`
- Ground truth: `datasets/gt/traj_lidar_outdoor_hard_01.txt`
- Output poses: `2859`

## Fair Config

Both configs are identical except for `registration_error_factor_type`:

- `config_fair_cpu/`: `VGICP`
- `config_fair_glil/`: `VGICP_CORESET`

Shared settings:

- `sub_mapping.enable_optimization=false`
- `sub_mapping.create_between_factors=false`
- `global_mapping.enable_imu=false`
- odometry: CPU
- preprocess: `num_threads=1`
- odometry threads: `registration=2`, `covariance=1`, `initialization=1`

## Important Correction

The historical `*_play1_*` runs are not valid `1.0x` replay evidence.

- Before 2026-04-15, `glil_rosbag` ignored CLI `--ros-args -p playback_speed:=...`
- So `outdoor_hard_01a_fair_cpu_threadsplitfix_play1_20260415` and `...glil_threadsplitfix_play1_20260415` still ran at roughly `4.5x .. 9.5x`
- Those directories should be treated as historical labels only, not as a reproducible replay contract

## Latest Clean-Finish Sample

These are the latest plain fair-config runs without the broken `playback_speed` interpretation:

| Metric | CPU VGICP | GLIL CORESET |
|---|---:|---:|
| Result dir | `outdoor_hard_01a_fair_cpu_cfgfix_20260415` | `outdoor_hard_01a_fair_glil_cfgfix_20260415` |
| APE RMSE (m) | 0.786651 | 0.788599 |
| Wall Time (sec) | 86.13 | 86.90 |
| Peak RSS (KB) | 462744 | 445572 |
| num_submaps | 8 | 8 |

Interpretation:

- Under the latest clean-finish sample, CPU and GLIL are effectively tied
- This does not establish a GLIL advantage in fair/full-global mode
- Because replay reproducibility is unresolved, treat these as sample runs, not official contract runs

## Actual `1.0x` Replay Check

After fixing the CLI override bug, actual full-bag `playback_speed:=1.0` CPU replays were still bad:

| Result dir | Config | num_submaps | APE RMSE (m) | Odom APE RMSE (m) | Wall Time (sec) |
|---|---|---:|---:|---:|---:|
| `outdoor_hard_01a_fair_cpu_odomdiag_fullplay1_20260415` | `config_fair_cpu_odomdiag` | 8 | 150.228687 | 149.922019 | 384.03 |
| `outdoor_hard_01a_fair_cpu_realplay1_20260415` | `config_fair_cpu` | 7 | 150.392872 | n/a | 382.90 |

Takeaway:

- Fixed replay speed is not a stabilizer by itself
- The earlier claim that `playback_speed:=1` made fair replay reproducible is no longer supported

## Variance Diagnosis

Confirmed variance sources:

1. preprocess-side multi-thread `randomgrid_sampling()`
2. odometry-side multi-thread covariance estimation / loose initial-state estimation

What is no longer supported:

- `playback_speed:=1` as a sufficient fix

### Good vs Bad Event Counts

The good fair sample and the bad actual-`1.0x` run have the same warning counts around the suspected failure region:

| Pattern | `cpu_cfgfix` | `cpu_realplay1` |
|---|---:|---:|
| `large time gap between consecutive IMU data!!` | 1 | 1 |
| `large time difference between points and imu!!` | 87 | 87 |
| `insufficient number of IMU data between LiDAR scans!! (odometry_estimation)` | 5 | 5 |
| `too few points in the filtered cloud (6 points)` | 1 | 1 |
| `too few points in the filtered cloud (0 points)` | 6 | 6 |

So the existence of the IMU-starvation / tiny-frame cluster is not enough to explain good vs bad outcomes.

### Trigger Region

Bad runs consistently show:

- `frame=877`
- `seq=900`
- `stamp=1694532945.900787`
- `filtered_points=6..7`

Then:

- `seq=901`: empty frame
- `seq=902`: sparse frame (`196` points)
- `estimated_v_norm`: jumps from about `1.66` to `5.37 m/s`

However, a `start_offset` window replay around the same stamp cluster stayed locally stable, so this cluster is a trigger, not a sufficient root cause.

## Current Interpretation

- GLIL fair/full-global superiority has not been reproduced yet
- The latest clean-finish fair sample is nearly identical between CPU and GLIL
- Actual `1.0x` replay does not fix the instability
- The next useful discriminator is low-overhead odometry-state tracing before the `seq=900` trigger, not more replay-speed sweeps

## Recommended Next Step

1. Compare good `outdoor_hard_01a_fair_cpu_cfgfix_20260415` and bad `outdoor_hard_01a_fair_cpu_realplay1_20260415`.
2. Instrument only event windows around `frame 807..811` and `frame 877..902`.
3. Use compact or event-triggered tracing instead of full per-frame logging, which can perturb the run.
4. Keep investigating why `num_matching_cost_factors` remains `0`.

## Result Paths

- CPU clean-finish sample: `results/outdoor_hard_01a_fair_cpu_cfgfix_20260415/`
- GLIL clean-finish sample: `results/outdoor_hard_01a_fair_glil_cfgfix_20260415/`
- CPU actual `1.0x` debug run: `results/outdoor_hard_01a_fair_cpu_odomdiag_fullplay1_20260415/`
- CPU actual `1.0x` plain run: `results/outdoor_hard_01a_fair_cpu_realplay1_20260415/`
