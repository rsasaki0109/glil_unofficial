# GLIL Comparison Report: CPU vs GLIL Coreset on outdoor_hard_01a

## Dataset
- **Sequence**: outdoor_hard_01a (Livox MID360, outdoor hard scenario)
- **Ground truth**: traj_lidar_outdoor_hard_01.txt (5147 poses)
- **Rosbag duration**: ~375 seconds (~6 min)
- **Output poses**: 2859 (both runs)

## Results Summary

| Metric                  | CPU (VGICP) | GLIL (VGICP_CORESET) | Upstream glim (reference) |
|-------------------------|-------------|----------------------|---------------------------|
| APE RMSE (m)            | 0.739       | 2.909                | 0.796                     |
| APE Mean (m)            | 0.612       | 2.642                | 0.679                     |
| APE Median (m)          | 0.495       | 2.412                | 0.544                     |
| APE Max (m)             | 2.476       | 6.584                | 2.529                     |
| APE Min (m)             | 0.080       | 0.198                | 0.098                     |
| APE Std (m)             | 0.414       | 1.218                | 0.415                     |
| Wall Time (sec)         | 192.6       | 203.0                | N/A (previous run)        |
| Peak RSS (MB)           | 1420        | 849                  | N/A                       |

All APE values computed with Sim(3) Umeyama alignment (`evo_ape tum --align --correct_scale`).

## Analysis

1. **Accuracy**: The CPU baseline (VGICP) achieves significantly better accuracy than the GLIL coreset variant. The RMSE is ~3.9x worse with coreset (0.739 m vs 2.909 m), and the max error more than doubles (2.476 m vs 6.584 m). The CPU baseline is very close to the upstream glim reference result.

2. **Timing**: Wall time is comparable (192.6 s vs 203.0 s). The GLIL coreset is ~5% slower despite the theoretical advantage of coresets for reducing computation. This may be due to the overhead of coreset construction or the sub_mapping optimization being enabled in the GLIL config (`enable_optimization: true` vs `false`).

3. **Memory**: The GLIL coreset uses significantly less memory (849 MB vs 1420 MB, a 40% reduction). This is the expected advantage of the coreset approach -- compressing the point cloud representation.

4. **Caveats**:
   - The global mapping ISAM2 optimizer crashed early in both runs due to the `IndeterminantLinearSystemException` at the initial large time gap in the dataset. After the crash, global loop closure was disabled for the remainder of each run, so both trajectories are effectively odometry + sub_mapping only (no global optimization).
   - The GLIL coreset config uses `enable_optimization: true` in sub_mapping and different voxel resolutions than the CPU config, which affects the comparison. These were the default GLIL configs.
   - The dataset has challenging sections with large LiDAR time gaps, empty point clouds, and IMU data dropouts, which required robustness patches to glil code.

## Config Details

### CPU Baseline (config_sub_mapping_cpu.json)
- registration_error_factor_type: VGICP
- enable_optimization: false
- keyframe_voxel_resolution: 0.25

### GLIL Coreset (config_sub_mapping_glil.json)
- registration_error_factor_type: VGICP_CORESET
- enable_optimization: true
- keyframe_voxel_resolution: 0.5

### Global Mapping (both)
- registration_error_factor_type: VGICP / VGICP_CORESET respectively
- enable_imu: true (GLIL) / false (CPU)
- ISAM2 crashed early in both runs; disabled for remainder

### Common Settings
- Odometry: CPU (GICP), same for both
- Sensor: Livox MID360, acc_scale=9.80665, identity T_lidar_imu
- QoS: sensor_data with IMU depth=1000

## Files
- CPU result: `<workspace>/results/outdoor_hard_01a_cpu_test/traj_lidar.txt`
- GLIL result: `<workspace>/results/outdoor_hard_01a_glil_test/traj_lidar.txt`
- Upstream reference: `<workspace>/results/latest_glim/outdoor_hard_01a_full_20260408_174639/traj_lidar.txt`
- Ground truth: `<workspace>/datasets/gt/traj_lidar_outdoor_hard_01.txt`
