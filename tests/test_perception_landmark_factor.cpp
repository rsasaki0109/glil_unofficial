// SPDX-License-Identifier: MIT
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <glil/factors/perception_landmark_factor.hpp>
#include <glil/perception/cloud_landmark_extractor.hpp>
#include <glil/perception/perception_factor_builder.hpp>
#include <glil/perception/perception_observation_io.hpp>

namespace {
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

constexpr double kPi = 3.14159265358979323846;

void expect_near(const gtsam::Vector3& actual, const gtsam::Vector3& expected, double tol) {
  const double err = (actual - expected).norm();
  if (err > tol) {
    std::cerr << "expected " << expected.transpose() << ", got " << actual.transpose() << ", err=" << err << std::endl;
    std::abort();
  }
}

void test_zero_error_and_jacobians() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  glil::PerceptionLandmarkFactor factor(X(0), L(0), gtsam::Point3(1.0, 0.0, 0.0), noise, 10.0, "pole", 42, 0.8);

  const gtsam::Pose3 pose;
  const gtsam::Point3 landmark(1.0, 0.0, 0.0);
  gtsam::Matrix H_pose;
  gtsam::Matrix H_landmark;
  const gtsam::Vector err = factor.evaluateError(pose, landmark, H_pose, H_landmark);

  expect_near(err, gtsam::Vector3::Zero(), 1e-12);
  assert(H_pose.rows() == 3 && H_pose.cols() == 6);
  assert(H_landmark.rows() == 3 && H_landmark.cols() == 3);
  assert(factor.class_id() == "pole");
  assert(factor.landmark_id() == 42);
}

void test_noise_from_observation() {
  glil::PerceptionObservation observation;
  observation.position_sensor = gtsam::Point3(1.0, 2.0, 3.0);
  observation.covariance.diagonal() << 0.04, 0.09, 0.16;
  observation.confidence = 0.25;
  assert(observation.valid());

  const auto noise = glil::make_perception_noise_model(observation);
  const gtsam::Vector whitened = noise->whiten(gtsam::Vector3(0.4, 0.6, 0.8));

  // confidence=0.25 doubles sigma, so the residuals whiten to 1.0.
  expect_near(whitened, gtsam::Vector3::Ones(), 1e-12);
}

void test_csv_loader_and_builder() {
  const std::string csv =
    "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_yy,cov_zz,confidence\n"
    "1.0,pole,7,2.0,0.0,0.0,0.04,0.04,0.09,0.8\n"
    "1.1,car,8,1.0,0.0,0.0,0.10,0.10,0.10,0.9\n"
    "1.2,sign,7,2.1,0.0,0.0,0.04,0.04,0.09,0.7\n";

  std::istringstream input(csv);
  const auto loaded = glil::load_perception_observations_csv(input);
  assert(loaded.errors.empty());
  assert(loaded.observations.size() == 3);
  assert(loaded.observations.front().class_id == "pole");

  glil::PerceptionObservation full_covariance;
  std::string error;
  assert(glil::parse_perception_observation_csv_row(
    "2.0,fiducial,9,1.0,2.0,3.0,0.1,0.01,0.02,0.01,0.2,0.03,0.02,0.03,0.3,0.95", full_covariance, &error));
  assert(std::abs(full_covariance.covariance(0, 1) - 0.01) < 1e-12);
  assert(std::abs(full_covariance.covariance(2, 2) - 0.3) < 1e-12);

  glil::PerceptionFactorBuilderParams params;
  params.allowed_class_ids = {"pole", "sign"};
  params.min_confidence = 0.5;
  params.robust_loss = "HUBER";
  params.robust_loss_width = 1.5;
  glil::PerceptionFactorBuilder builder(params);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values pending_values;
  const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(10.0, 0.0, 0.0));
  const auto summary = builder.add_observations(graph, pending_values, X(3), pose, loaded.observations);

  assert(summary.accepted == 2);
  assert(summary.rejected_class == 1);
  assert(summary.inserted_landmarks == 1);
  assert(summary.reused_landmarks == 1);
  assert(graph.size() == 2);
  const auto perception_factor = std::dynamic_pointer_cast<glil::PerceptionLandmarkFactor>(graph.at(0));
  assert(perception_factor);
  const auto robust_noise = std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(perception_factor->noiseModel());
  assert(robust_noise);
  const auto huber_loss = std::dynamic_pointer_cast<gtsam::noiseModel::mEstimator::Huber>(robust_noise->robust());
  assert(huber_loss);
  assert(std::abs(huber_loss->modelParameter() - 1.5) < 1e-12);
  assert(pending_values.exists(L(7)));
  assert(!pending_values.exists(L(8)));
  expect_near(pending_values.at<gtsam::Point3>(L(7)), gtsam::Point3(12.0, 0.0, 0.0), 1e-12);
}

void test_csv_loader_warnings() {
  const std::string csv =
    "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_yy,cov_zz,confidence\n"
    "1.0,pole,7,2.0,0.0,0.0,0.04,0.04,0.09,0.8\n"
    "1.0,pole,7,2.0,0.0,0.0,0.04,0.04,0.09,0.8\n"
    "1.1,sign,7,2.1,0.0,0.0,0.04,0.04,0.09,0.7\n"
    "1.2,pole,8,5.0,0.0,0.0,0.04,0.04,0.0,0.9\n";

  std::istringstream input(csv);
  const auto loaded = glil::load_perception_observations_csv(input);
  assert(loaded.errors.empty());
  assert(loaded.observations.size() == 4);

  int duplicate_stamp = 0;
  int class_collision = 0;
  int degenerate_cov = 0;
  int non_pd_cov = 0;
  for (const auto& warning : loaded.warnings) {
    switch (warning.kind) {
      case glil::PerceptionObservationCsvWarning::Kind::DuplicateStampLandmark:
        duplicate_stamp++;
        assert(warning.observation_index == 1);
        assert(warning.landmark_id == 7);
        break;
      case glil::PerceptionObservationCsvWarning::Kind::LandmarkClassCollision:
        class_collision++;
        assert(warning.observation_index == 2);
        assert(warning.landmark_id == 7);
        assert(warning.class_id == "sign");
        break;
      case glil::PerceptionObservationCsvWarning::Kind::DegenerateCovariance:
        degenerate_cov++;
        assert(warning.observation_index == 3);
        assert(warning.landmark_id == 8);
        break;
      case glil::PerceptionObservationCsvWarning::Kind::NonPositiveDefiniteCovariance:
        non_pd_cov++;
        break;
    }
  }
  assert(duplicate_stamp == 1);
  assert(class_collision == 1);
  assert(degenerate_cov == 1);
  assert(non_pd_cov == 0);

  // Asymmetric full 3x3 covariance should trigger non_pd_cov warning.
  const std::string asymmetric_csv =
    "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz,confidence\n"
    "3.0,pole,10,1.0,0.0,0.0,0.1,0.05,0.0,-0.05,0.1,0.0,0.0,0.0,0.1,0.9\n";
  std::istringstream asymmetric_input(asymmetric_csv);
  const auto asymmetric = glil::load_perception_observations_csv(asymmetric_input);
  assert(asymmetric.errors.empty());
  assert(asymmetric.observations.size() == 1);
  int asymmetric_warnings = 0;
  for (const auto& warning : asymmetric.warnings) {
    if (warning.kind == glil::PerceptionObservationCsvWarning::Kind::NonPositiveDefiniteCovariance) {
      asymmetric_warnings++;
      assert(warning.observation_index == 0);
      assert(warning.landmark_id == 10);
    }
  }
  assert(asymmetric_warnings == 1);

  // Symmetric but indefinite covariance should also trigger the warning.
  const std::string indefinite_csv =
    "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz,confidence\n"
    "4.0,pole,11,1.0,0.0,0.0,0.1,0.5,0.0,0.5,0.1,0.0,0.0,0.0,0.1,0.9\n";
  std::istringstream indefinite_input(indefinite_csv);
  const auto indefinite = glil::load_perception_observations_csv(indefinite_input);
  assert(indefinite.errors.empty());
  assert(indefinite.observations.size() == 1);
  int indefinite_warnings = 0;
  for (const auto& warning : indefinite.warnings) {
    if (warning.kind == glil::PerceptionObservationCsvWarning::Kind::NonPositiveDefiniteCovariance) {
      indefinite_warnings++;
      assert(warning.landmark_id == 11);
    }
  }
  assert(indefinite_warnings == 1);
}

void test_csv_loader_v2_header() {
  // Same observations expressed twice: v1 positional schema vs v2 header with
  // optional schema_version / sensor_frame_id / detection_score / track_age /
  // source columns plus an unknown `extra_metadata` column. Both shapes must
  // produce identical observations so writers can move to v2 without breaking
  // existing consumers.
  const std::string v1 =
    "stamp,class_id,landmark_id,x,y,z,cov_xx,cov_yy,cov_zz,confidence\n"
    "1.0,pole,7,2.0,0.0,0.0,0.04,0.04,0.09,0.8\n"
    "1.1,sign,8,2.1,0.0,0.0,0.04,0.04,0.09,0.9\n";
  const std::string v2 =
    "schema_version,stamp,class_id,landmark_id,sensor_frame_id,x,y,z,cov_xx,cov_yy,cov_zz,"
    "confidence,detection_score,tracking_score,track_age,source,extra_metadata\n"
    "2,1.0,pole,7,lidar_front,2.0,0.0,0.0,0.04,0.04,0.09,0.8,0.85,0.7,3,tracker_a,freeform\n"
    "2,1.1,sign,8,lidar_front,2.1,0.0,0.0,0.04,0.04,0.09,0.9,0.88,0.72,5,tracker_a,freeform\n";

  std::istringstream v1_input(v1);
  std::istringstream v2_input(v2);
  const auto loaded_v1 = glil::load_perception_observations_csv(v1_input);
  const auto loaded_v2 = glil::load_perception_observations_csv(v2_input);
  assert(loaded_v1.errors.empty());
  assert(loaded_v2.errors.empty());
  assert(loaded_v1.observations.size() == 2);
  assert(loaded_v2.observations.size() == 2);

  for (std::size_t i = 0; i < 2; ++i) {
    const auto& a = loaded_v1.observations[i];
    const auto& b = loaded_v2.observations[i];
    assert(std::abs(a.stamp - b.stamp) < 1e-12);
    assert(a.class_id == b.class_id);
    assert(a.landmark_id == b.landmark_id);
    for (int axis = 0; axis < 3; ++axis) {
      assert(std::abs(a.position_sensor[axis] - b.position_sensor[axis]) < 1e-12);
      assert(std::abs(a.covariance(axis, axis) - b.covariance(axis, axis)) < 1e-12);
    }
    assert(std::abs(a.confidence - b.confidence) < 1e-12);
  }

  // Reordered columns + full 3x3 covariance via header mapping.
  const std::string v2_full =
    "landmark_id,stamp,class_id,x,y,z,cov_xx,cov_xy,cov_xz,cov_yx,cov_yy,cov_yz,cov_zx,cov_zy,cov_zz,confidence,schema_version\n"
    "9,2.0,fiducial,1.0,2.0,3.0,0.1,0.01,0.02,0.01,0.2,0.03,0.02,0.03,0.3,0.95,2\n";
  std::istringstream v2_full_input(v2_full);
  const auto loaded_full = glil::load_perception_observations_csv(v2_full_input);
  assert(loaded_full.errors.empty());
  assert(loaded_full.observations.size() == 1);
  const auto& full_obs = loaded_full.observations.front();
  assert(full_obs.landmark_id == 9);
  assert(full_obs.class_id == "fiducial");
  assert(std::abs(full_obs.covariance(0, 1) - 0.01) < 1e-12);
  assert(std::abs(full_obs.covariance(2, 2) - 0.3) < 1e-12);
}

void test_cloud_landmark_extractor() {
  const std::vector<gtsam::Point3> world_points = {
    gtsam::Point3(0.10, 0.10, 0.10),
    gtsam::Point3(0.20, 0.10, 0.10),
    gtsam::Point3(0.10, 0.20, 0.10),
    gtsam::Point3(0.20, 0.20, 0.10),
    gtsam::Point3(2.10, 0.00, 0.00),
    gtsam::Point3(2.20, 0.00, 0.00),
    gtsam::Point3(4.00, 0.00, 0.00),
  };

  const gtsam::Pose3 pose_a;
  const gtsam::Pose3 pose_b(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));

  std::vector<gtsam::Point3> sensor_a;
  std::vector<gtsam::Point3> sensor_b;
  for (const auto& point_world : world_points) {
    sensor_a.push_back(pose_a.transformTo(point_world));
    sensor_b.push_back(pose_b.transformTo(point_world));
  }

  glil::CloudLandmarkExtractorParams params;
  params.voxel_resolution = 1.0;
  params.min_points_per_landmark = 2;
  params.min_covariance = 1e-4;
  params.confidence_point_count = 4.0;
  params.class_id = "cloud_voxel";

  const auto extracted_a = glil::extract_cloud_landmark_observations(sensor_a, pose_a, 4.2, params);
  const auto extracted_b = glil::extract_cloud_landmark_observations(sensor_b, pose_b, 4.2, params);

  assert(extracted_a.input_points == world_points.size());
  assert(extracted_a.finite_points == world_points.size());
  assert(extracted_a.voxel_candidates == 3);
  assert(extracted_a.rejected_min_points == 1);
  assert(extracted_a.observations.size() == 2);
  assert(extracted_b.observations.size() == 2);

  std::vector<std::uint64_t> ids_a;
  std::vector<std::uint64_t> ids_b;
  for (std::size_t i = 0; i < extracted_a.observations.size(); i++) {
    const auto& observation_a = extracted_a.observations[i];
    const auto& observation_b = extracted_b.observations[i];
    assert(observation_a.valid());
    assert(observation_a.class_id == "cloud_voxel");
    assert(observation_a.covariance(0, 0) >= params.min_covariance);
    assert(observation_a.covariance(1, 1) >= params.min_covariance);
    assert(observation_a.covariance(2, 2) >= params.min_covariance);
    ids_a.push_back(observation_a.landmark_id);
    ids_b.push_back(observation_b.landmark_id);
  }
  assert(ids_a == ids_b);

  glil::PerceptionFactorBuilderParams builder_params;
  builder_params.allowed_class_ids = {"cloud_voxel"};
  glil::PerceptionFactorBuilder builder(builder_params);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  const auto summary = builder.add_observations(graph, values, X(5), pose_a, extracted_a.observations);
  assert(summary.accepted == 2);
  assert(summary.inserted_landmarks == 2);
  assert(graph.size() == 2);

  params.max_landmarks = 1;
  const auto capped = glil::extract_cloud_landmark_observations(sensor_a, pose_a, 4.2, params);
  assert(capped.observations.size() == 1);
  assert(std::abs(capped.observations.front().confidence - 1.0) < 1e-12);
}

void test_landmark_optimization() {
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.0, 0.0, kPi / 2.0), gtsam::Point3(1.0, 2.0, 3.0));
  const gtsam::Point3 measured_sensor(2.0, 0.0, 0.0);
  const gtsam::Point3 true_landmark = pose.transformFrom(measured_sensor);

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), pose, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.emplace_shared<glil::PerceptionLandmarkFactor>(X(0), L(0), measured_sensor, gtsam::noiseModel::Isotropic::Sigma(3, 0.05));

  gtsam::Values initial;
  initial.insert(X(0), pose);
  initial.insert(L(0), true_landmark + gtsam::Point3(0.4, -0.3, 0.2));

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  expect_near(result.at<gtsam::Point3>(L(0)), true_landmark, 1e-6);
}
}  // namespace

int main() {
  test_zero_error_and_jacobians();
  test_noise_from_observation();
  test_csv_loader_and_builder();
  test_csv_loader_warnings();
  test_csv_loader_v2_header();
  test_cloud_landmark_extractor();
  test_landmark_optimization();
  return 0;
}
