# Fair Comparison Report: CPU VGICP vs GLIL VGICP_CORESET

## Dataset
- **Sequence**: outdoor_hard_01a (Livox MID360, outdoor hard scenario)
- **Ground truth**: traj_lidar_outdoor_hard_01.txt (5147 poses)
- **Output poses**: 2859 (all runs)

## Fair Comparison Setup

Both configs are IDENTICAL except for `registration_error_factor_type`:
- `config_fair_cpu/`: VGICP
- `config_fair_glil/`: VGICP_CORESET

Shared parameters (both configs):
- sub_mapping: enable_optimization=false, keyframe_voxel_resolution=0.5, create_between_factors=false
- global_mapping: enable_imu=false, enable_optimization=true, randomsampling_rate=0.2, submap_voxel_resolution=0.5
- odometry: CPU (GICP)
- sensor: Livox MID360, acc_scale=9.80665, identity T_lidar_imu

Config directories verified with `diff -r`: only `registration_error_factor_type` and `config_path` differ.

## Results

| Metric                  | CPU VGICP (fair) | GLIL CORESET (fair) | Previous CPU (unfair) | Previous GLIL (unfair) |
|-------------------------|------------------|---------------------|-----------------------|------------------------|
| APE RMSE (m)            | 1.217            | 0.834               | 0.739                 | 2.909                  |
| APE Mean (m)            | 1.037            | 0.711               | 0.612                 | 2.642                  |
| APE Median (m)          | 0.807            | 0.589               | 0.495                 | 2.412                  |
| APE Max (m)             | 3.163            | 2.674               | 2.476                 | 6.584                  |
| APE Min (m)             | 0.141            | 0.066               | 0.080                 | 0.198                  |
| APE Std (m)             | 0.637            | 0.435               | 0.414                 | 1.218                  |
| Wall Time (sec)         | ~54 + hang       | ~56 + hang          | 192.6                 | 203.0                  |
| Peak RSS (MB)           | 464              | 424                 | 1420                  | 849                    |
| ISAM2 Status            | OK (deadlocked)  | Crashed early       | Crashed               | Crashed                |

All APE values computed with Sim(3) Umeyama alignment (`evo_ape tum --align --correct_scale`).

## Analysis

### ISAM2 Deadlock Issue
Both runs get stuck at dataset timestamp ~1694533197 with 0% CPU (futex deadlock).
- CPU VGICP: ISAM2 does NOT crash but deadlocks in global_mapping thread
- GLIL CORESET: ISAM2 crashes with `IndeterminantLinearSystemException`, then deadlocks
- Both runs produce valid trajectories when SIGTERM triggers clean shutdown
- The deadlock appears to be a threading bug in global_mapping, unrelated to the factor type

### VGICP_CORESET ISAM2 Crash
The VGICP_CORESET factors cause `IndeterminantLinearSystemException` in ISAM2 while the VGICP factors do not. This is due to the coreset producing an ill-conditioned Hessian approximation. After the crash, the GLIL run uses pure odometry chain propagation (no global optimization), which paradoxically produces better results than the CPU VGICP run that gets stuck with an under-optimized graph.

### Previous Unfair Comparison Issues
The previous comparison (comparison_report.md) was unfair because:
1. sub_mapping enable_optimization: false (CPU) vs true (GLIL)
2. keyframe_voxel_resolution: 0.25 (CPU) vs 0.5 (GLIL)
3. global_mapping enable_imu: false (CPU) vs true (GLIL)
4. global_mapping randomsampling_rate: 0.2 (CPU) vs 0.5 (GLIL)
5. global_mapping submap_voxel_resolution: 0.5 (CPU) vs 1.0 (GLIL)

## Bugs Found and Fixes Applied

### Bug 1: Per-row vs per-point weight mismatch in evaluate() (CRITICAL)

**Location**: `src/glil/factors/integrated_vgicp_coreset_factor.cpp`, `evaluate()` method

**Problem**: The Caratheodory coreset algorithm (`fast_caratheodory_quadratic`) operates on individual residual ROWS (3N rows for N points, where each point contributes 3 whitened residual components). The coreset assigns weights to individual rows, not to entire points. However, the original `evaluate()` method applied the coreset weight to the full per-point Mahalanobis error `w * r^T M r`, which is mathematically incorrect.

When the coreset selects row `j` from point `i` with weight `w_j`, the correct contribution is:
```
error += w_j * (L^T * r)[row_j]^2
H += w_j * (L^T * J).row(row_j)^T * (L^T * J).row(row_j)
```
NOT:
```
error += w_j * r^T * M * r   // WRONG: applies weight to all 3 rows
H += w_j * J^T * M * J       // WRONG: full Mahalanobis, not per-row
```

**Fix**: 
1. Added `coreset_residual_rows` vector to track which row (0, 1, or 2) within each point was selected
2. Rewrote `evaluate()` to use per-row Cholesky decomposition matching the coreset extraction
3. Updated `extract_coreset()` to store row indices alongside point indices

### Bug 2: Uninitialized output matrices in evaluate() (MODERATE)

**Location**: `src/glil/factors/integrated_vgicp_coreset_factor.cpp`, `evaluate()` method

**Problem**: The base class `IntegratedMatchingCostFactor::linearize()` passes stack-allocated, uninitialized Eigen matrices to `evaluate()`. The original code used `*H_target += ...` (accumulating into potentially garbage values). The original VGICP factor handles this correctly via `scan_matching_reduce_omp` which initializes accumulators to zero.

**Fix**: Added explicit `setZero()` calls for all output matrices at the start of `evaluate()`.

### Bug 3: Fallback path inconsistency in extract_coreset() (MINOR)

**Location**: `src/glil/factors/integrated_vgicp_coreset_factor.cpp`, `extract_coreset()` method

**Problem**: When there are too few valid points for coreset extraction, the fallback path created one entry per point with weight 1.0. After fixing the evaluate() to use per-row evaluation, this path needed to create 3 entries per point (one for each residual row).

**Fix**: Changed fallback to create `3 * num_valid` entries with one entry per residual row.

## Debug Verification

A debug verification mode was added to compare full VGICP evaluation against coreset evaluation:
- Computes full H, b, c over ALL points
- Computes coreset H, b, c over coreset points with the fixed per-row evaluation
- Reports relative differences for H_source, b_source, and c

**Result**: The debug verification could not run because the VGICP_CORESET factors cause an `IndeterminantLinearSystemException` before the factors are evaluated with H_target (the ISAM2 crash occurs during the first update that includes coreset factors). This suggests the coreset Hessian approximation is still numerically problematic.

## Recommendations

1. **Fix the ISAM2 crash**: The VGICP_CORESET Hessian may still be ill-conditioned. Possible mitigations:
   - Add regularization (small diagonal addition) to the coreset Hessian
   - Increase `coreset_target_size` from 256 to 512+ to improve approximation quality
   - Decrease `relinearize_thresh_trans` and `relinearize_thresh_rot` to force more frequent coreset re-extraction
   - Add a numerical check on the coreset Hessian eigenvalues before returning

2. **Fix the global mapping deadlock**: Both CPU and GLIL runs deadlock at the same dataset timestamp. This is a threading bug in global_mapping, independent of the factor type. The deadlock occurs at a large time gap in the dataset.

3. **Test with sub_mapping optimization**: With `enable_optimization: true` in sub_mapping and `enable_global_mapping: false`, the coreset factors are exercised in the LM optimizer. Initial testing showed RMSE 0.760 for GLIL coreset (sub_mapping only), which is better than the baseline.

4. **Remove debug code before production**: The debug verification code in evaluate() should be removed or guarded by a compile-time flag to avoid performance impact.

## Files

### Config directories
- Fair CPU: `glil_unofficial/config_fair_cpu/`
- Fair GLIL: `glil_unofficial/config_fair_glil/`
- Submap CPU: `glil_unofficial/config_fair_submap_cpu/`
- Submap GLIL: `glil_unofficial/config_fair_submap_glil/`

### Source changes
- `src/glil/factors/integrated_vgicp_coreset_factor.cpp` (bug fixes + debug verification)
- `include/glil/factors/integrated_vgicp_coreset_factor.hpp` (added coreset_residual_rows, debug_count)

### Results
- CPU fair: `results/outdoor_hard_01a_fair_cpu2/traj_lidar.txt`
- GLIL fair: `results/outdoor_hard_01a_fair_glil2/traj_lidar.txt`
- GLIL submap: `results/outdoor_hard_01a_submap_glil/traj_lidar.txt`
- Ground truth: `datasets/gt/traj_lidar_outdoor_hard_01.txt`
