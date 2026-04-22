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

## C++ Example

```cpp
#include <glil/factors/perception_landmark_factor.hpp>

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

glil::PerceptionObservation obs;
obs.stamp = 12.3;
obs.class_id = "pole";
obs.landmark_id = 42;
obs.position_sensor = gtsam::Point3(4.0, 0.2, 1.1);
obs.covariance = gtsam::Matrix3::Identity() * 0.04;
obs.confidence = 0.8;

auto noise = glil::make_perception_noise_model(obs);
graph.emplace_shared<glil::PerceptionLandmarkFactor>(X(keyframe_id), L(obs.landmark_id), obs, noise);

if (!values.exists(L(obs.landmark_id))) {
  const auto landmark_world = current_pose.transformFrom(obs.position_sensor);
  values.insert(L(obs.landmark_id), landmark_world);
}
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
- a CTest smoke test that checks residuals, Jacobians, confidence-weighted noise,
  and landmark optimization

Not implemented yet:

- ROS/ROS 2 perception message adapters
- JSON/CSV observation loader
- data association and landmark lifecycle policy
- robust-kernel configuration for outlier-heavy detectors
