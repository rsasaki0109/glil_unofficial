// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <glil/perception/perception_observation.hpp>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/string.hpp>
#endif

namespace glil {

/**
 * @brief Factor from a pose to a perceived 3D landmark.
 *
 * The residual is:
 *
 *   pose.transformTo(landmark_world) - measured_position_sensor
 *
 * This keeps perception frontend outputs decoupled from the mapping backend: an
 * external detector/tracker only needs to provide a stable landmark ID and a 3D
 * position in the sensor frame.
 */
class PerceptionLandmarkFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>;
  using shared_ptr = std::shared_ptr<PerceptionLandmarkFactor>;
  using Base::evaluateError;

  PerceptionLandmarkFactor() = default;

  PerceptionLandmarkFactor(
    gtsam::Key pose_key,
    gtsam::Key landmark_key,
    const gtsam::Point3& measured_position_sensor,
    const gtsam::SharedNoiseModel& noise_model,
    double stamp = 0.0,
    std::string class_id = {},
    std::uint64_t landmark_id = 0,
    double confidence = 1.0);

  PerceptionLandmarkFactor(
    gtsam::Key pose_key,
    gtsam::Key landmark_key,
    const PerceptionObservation& observation,
    const gtsam::SharedNoiseModel& noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Pose3& pose,
    const gtsam::Point3& landmark_world,
    gtsam::OptionalMatrixType H_pose,
    gtsam::OptionalMatrixType H_landmark) const override;

  gtsam::NonlinearFactor::shared_ptr clone() const override;

  void print(const std::string& s, const gtsam::KeyFormatter& key_formatter = gtsam::DefaultKeyFormatter) const override;
  bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;

  const gtsam::Point3& measured_position_sensor() const { return measured_position_sensor_; }
  double stamp() const { return stamp_; }
  const std::string& class_id() const { return class_id_; }
  std::uint64_t landmark_id() const { return landmark_id_; }
  double confidence() const { return confidence_; }

private:
  gtsam::Point3 measured_position_sensor_ = gtsam::Point3::Zero();
  double stamp_ = 0.0;
  std::string class_id_;
  std::uint64_t landmark_id_ = 0;
  double confidence_ = 1.0;

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(measured_position_sensor_);
    ar& BOOST_SERIALIZATION_NVP(stamp_);
    ar& BOOST_SERIALIZATION_NVP(class_id_);
    ar& BOOST_SERIALIZATION_NVP(landmark_id_);
    ar& BOOST_SERIALIZATION_NVP(confidence_);
  }
#endif
};

}  // namespace glil
