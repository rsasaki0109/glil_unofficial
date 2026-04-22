// SPDX-License-Identifier: MIT
#include <glil/util/extension_module.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/isam2_ext.hpp>

#include <glil/mapping/callbacks.hpp>
#include <glil/perception/perception_factor_builder.hpp>
#include <glil/perception/perception_observation_io.hpp>
#include <glil/util/config.hpp>
#include <glil/util/logging.hpp>

namespace glil {
namespace {

constexpr const char* kConfigModule = "perception_csv_injector";

bool is_absolute_path(const std::string& path) {
  return !path.empty() && path.front() == '/';
}

std::string resolve_config_relative_path(const std::string& path) {
  if (path.empty() || is_absolute_path(path)) {
    return path;
  }

  const std::string config_path = GlobalConfig::instance()->param<std::string>("global", "config_path", ".");
  return config_path + "/" + path;
}

double origin_stamp(const SubMap::ConstPtr& submap) {
  if (!submap) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (!submap->frames.empty() && submap->origin_frame()) {
    return submap->origin_frame()->stamp;
  }
  if (!submap->odom_frames.empty() && submap->origin_odom_frame()) {
    return submap->origin_odom_frame()->stamp;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

class PerceptionCsvInjector : public ExtensionModule {
public:
  PerceptionCsvInjector() : logger_(create_module_logger("perception_csv")) {
    Config config(GlobalConfig::get_config_path("config_perception"));
    enabled_ = config.param<bool>(kConfigModule, "enabled", false);
    if (!enabled_) {
      logger_->info("perception CSV injector disabled");
      return;
    }

    time_tolerance_ = config.param<double>(kConfigModule, "time_tolerance", 0.05);
    consume_once_ = config.param<bool>(kConfigModule, "consume_once", true);
    const bool skip_header = config.param<bool>(kConfigModule, "skip_header", true);
    const bool skip_invalid_rows = config.param<bool>(kConfigModule, "skip_invalid_rows", false);
    const bool fail_on_load_error = config.param<bool>(kConfigModule, "fail_on_load_error", false);

    PerceptionFactorBuilderParams builder_params;
    builder_params.min_confidence = config.param<double>(kConfigModule, "min_confidence", 0.0);
    builder_params.min_sigma = config.param<double>(kConfigModule, "min_sigma", 1e-3);
    builder_params.min_noise_confidence = config.param<double>(kConfigModule, "min_noise_confidence", 1e-3);
    builder_params.initialize_missing_landmarks = config.param<bool>(kConfigModule, "initialize_missing_landmarks", true);

    const std::string landmark_symbol = config.param<std::string>(kConfigModule, "landmark_symbol", "l");
    if (!landmark_symbol.empty()) {
      builder_params.landmark_symbol = landmark_symbol.front();
    }

    for (const auto& class_id : config.param<std::vector<std::string>>(kConfigModule, "allowed_class_ids", {})) {
      builder_params.allowed_class_ids.insert(class_id);
    }
    for (const auto& class_id : config.param<std::vector<std::string>>(kConfigModule, "rejected_class_ids", {})) {
      builder_params.rejected_class_ids.insert(class_id);
    }
    builder_ = PerceptionFactorBuilder(builder_params);

    const std::string csv_path = resolve_config_relative_path(config.param<std::string>(kConfigModule, "csv_path", ""));
    if (csv_path.empty()) {
      logger_->warn("perception CSV injector enabled but csv_path is empty");
      enabled_ = false;
      return;
    }

    PerceptionObservationCsvOptions csv_options;
    csv_options.skip_header = skip_header;
    csv_options.skip_invalid_rows = skip_invalid_rows;
    csv_options.min_confidence = builder_params.min_confidence;

    auto loaded = load_perception_observations_csv(csv_path, csv_options);
    if (!loaded.errors.empty()) {
      const auto& first_error = loaded.errors.front();
      logger_->warn(
        "loaded perception CSV with {} errors; first error line={} reason={} text={}",
        loaded.errors.size(),
        first_error.line,
        first_error.message,
        first_error.text);
      if (fail_on_load_error) {
        enabled_ = false;
        return;
      }
    }

    observations_ = std::move(loaded.observations);
    std::sort(observations_.begin(), observations_.end(), [](const auto& lhs, const auto& rhs) { return lhs.stamp < rhs.stamp; });
    consumed_.assign(observations_.size(), false);

    logger_->info(
      "perception CSV injector loaded {} observations from {} tolerance={}s consume_once={}",
      observations_.size(),
      csv_path,
      time_tolerance_,
      consume_once_);

    insert_submap_callback_id_ = GlobalMappingCallbacks::on_insert_submap.add([this](const SubMap::ConstPtr& submap) { on_insert_submap(submap); });
    smoother_update_callback_id_ = GlobalMappingCallbacks::on_smoother_update.add(
      [this](gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) { on_smoother_update(isam2, new_factors, new_values); });
  }

  ~PerceptionCsvInjector() override {
    if (insert_submap_callback_id_ >= 0) {
      GlobalMappingCallbacks::on_insert_submap.remove(insert_submap_callback_id_);
    }
    if (smoother_update_callback_id_ >= 0) {
      GlobalMappingCallbacks::on_smoother_update.remove(smoother_update_callback_id_);
    }
  }

private:
  void on_insert_submap(const SubMap::ConstPtr& submap) {
    if (!enabled_ || !submap) {
      return;
    }

    const double stamp = origin_stamp(submap);
    if (!std::isfinite(stamp)) {
      logger_->debug("skip submap {} perception association because origin stamp is invalid", submap->id);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    submap_stamps_[submap->id] = stamp;
  }

  std::vector<std::size_t> find_observations_near(double stamp) const {
    std::vector<std::size_t> matches;
    if (observations_.empty() || time_tolerance_ < 0.0) {
      return matches;
    }

    const double lower_stamp = stamp - time_tolerance_;
    const double upper_stamp = stamp + time_tolerance_;
    const auto lower = std::lower_bound(observations_.begin(), observations_.end(), lower_stamp, [](const PerceptionObservation& observation, double value) {
      return observation.stamp < value;
    });

    for (auto itr = lower; itr != observations_.end() && itr->stamp <= upper_stamp; ++itr) {
      const std::size_t index = static_cast<std::size_t>(std::distance(observations_.begin(), itr));
      if (consume_once_ && consumed_[index]) {
        continue;
      }
      matches.push_back(index);
    }
    return matches;
  }

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    if (!enabled_ || observations_.empty()) {
      return;
    }

    PerceptionFactorBuilderResult total;
    std::size_t matched_observations = 0;
    std::size_t pose_count = 0;
    gtsam::Values existing_values;
    bool existing_values_loaded = false;

    struct PendingPose {
      gtsam::Key key;
      double stamp;
      gtsam::Pose3 pose_world_sensor;
    };

    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PendingPose> pending_poses;
    for (const auto& value : new_values) {
      const gtsam::Key pose_key = value.key;
      const gtsam::Symbol pose_symbol(pose_key);
      if (pose_symbol.chr() != 'x') {
        continue;
      }

      const auto stamp_itr = submap_stamps_.find(static_cast<int>(pose_symbol.index()));
      if (stamp_itr == submap_stamps_.end()) {
        continue;
      }

      try {
        pending_poses.push_back(PendingPose{pose_key, stamp_itr->second, new_values.at<gtsam::Pose3>(pose_key)});
      } catch (const std::exception& e) {
        logger_->warn("failed to read pending pose {} for perception injection: {}", pose_symbol.string(), e.what());
      }
    }

    for (const auto& pending_pose : pending_poses) {
      const std::vector<std::size_t> observation_indices = find_observations_near(pending_pose.stamp);
      if (observation_indices.empty()) {
        continue;
      }

      if (!existing_values_loaded) {
        try {
          existing_values = isam2.calculateEstimate();
        } catch (const std::exception& e) {
          logger_->debug("failed to get existing values for perception landmark reuse: {}", e.what());
        }
        existing_values_loaded = true;
      }

      std::vector<PerceptionObservation> selected;
      selected.reserve(observation_indices.size());
      for (const std::size_t index : observation_indices) {
        selected.push_back(observations_[index]);
        if (consume_once_) {
          consumed_[index] = true;
        }
      }

      total += builder_.add_observations(new_factors, new_values, pending_pose.key, pending_pose.pose_world_sensor, selected, &existing_values);
      matched_observations += selected.size();
      pose_count++;
    }

    if (matched_observations > 0) {
      logger_->info(
        "perception CSV injection poses={} matched={} accepted={} inserted_landmarks={} reused_landmarks={} rejected_invalid={} rejected_class={} rejected_missing_landmark={}",
        pose_count,
        matched_observations,
        total.accepted,
        total.inserted_landmarks,
        total.reused_landmarks,
        total.rejected_invalid,
        total.rejected_class,
        total.rejected_missing_landmark);
    }
  }

private:
  bool enabled_ = false;
  double time_tolerance_ = 0.05;
  bool consume_once_ = true;

  PerceptionFactorBuilder builder_;
  std::vector<PerceptionObservation> observations_;
  std::vector<bool> consumed_;
  std::unordered_map<int, double> submap_stamps_;
  mutable std::mutex mutex_;

  int insert_submap_callback_id_ = -1;
  int smoother_update_callback_id_ = -1;

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace glil

extern "C" glil::ExtensionModule* create_extension_module() {
  return new glil::PerceptionCsvInjector();
}
