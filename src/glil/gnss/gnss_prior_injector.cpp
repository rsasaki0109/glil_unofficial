// SPDX-License-Identifier: MIT
#include <glil/util/extension_module.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/isam2_ext.hpp>

#include <glil/gnss/gnss_observation_io.hpp>
#include <glil/mapping/callbacks.hpp>
#include <glil/util/config.hpp>
#include <glil/util/logging.hpp>

namespace glil {
namespace {

constexpr const char* kConfigModule = "gnss_prior_injector";

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

class GNSSPriorInjector : public ExtensionModule {
public:
  GNSSPriorInjector() : logger_(create_module_logger("gnss_csv")) {
    Config config(GlobalConfig::get_config_path("config_gnss"));
    enabled_ = config.param<bool>(kConfigModule, "enabled", false);
    if (!enabled_) {
      logger_->info("GNSS prior injector disabled");
      return;
    }

    time_tolerance_ = config.param<double>(kConfigModule, "time_tolerance", 0.05);
    consume_once_ = config.param<bool>(kConfigModule, "consume_once", true);
    min_sigma_ = config.param<double>(kConfigModule, "min_sigma", 1e-3);
    const bool skip_header = config.param<bool>(kConfigModule, "skip_header", true);
    const bool skip_invalid_rows = config.param<bool>(kConfigModule, "skip_invalid_rows", false);
    const bool fail_on_load_error = config.param<bool>(kConfigModule, "fail_on_load_error", false);

    const std::string csv_path = resolve_config_relative_path(config.param<std::string>(kConfigModule, "csv_path", ""));
    if (csv_path.empty()) {
      logger_->warn("GNSS prior injector enabled but csv_path is empty");
      enabled_ = false;
      return;
    }

    GNSSObservationCsvOptions csv_options;
    csv_options.skip_header = skip_header;
    csv_options.skip_invalid_rows = skip_invalid_rows;

    auto loaded = load_gnss_observations_csv(csv_path, csv_options);
    if (!loaded.errors.empty()) {
      const auto& first_error = loaded.errors.front();
      logger_->warn(
        "loaded GNSS CSV with {} errors; first error line={} reason={} text={}",
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
      "GNSS prior injector loaded {} observations from {} tolerance={}s consume_once={} min_sigma={}",
      observations_.size(),
      csv_path,
      time_tolerance_,
      consume_once_,
      min_sigma_);

    insert_submap_callback_id_ = GlobalMappingCallbacks::on_insert_submap.add([this](const SubMap::ConstPtr& submap) { on_insert_submap(submap); });
    smoother_update_callback_id_ = GlobalMappingCallbacks::on_smoother_update.add(
      [this](gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) { on_smoother_update(isam2, new_factors, new_values); });
  }

  ~GNSSPriorInjector() override {
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
      logger_->debug("skip submap {} GNSS association because origin stamp is invalid", submap->id);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    submap_stamps_[submap->id] = stamp;
  }

  int find_nearest_observation(double stamp) const {
    if (observations_.empty() || time_tolerance_ < 0.0) {
      return -1;
    }

    auto it = std::lower_bound(observations_.begin(), observations_.end(), stamp, [](const GNSSObservation& obs, double value) {
      return obs.stamp < value;
    });

    int best_index = -1;
    double best_delta = std::numeric_limits<double>::infinity();
    auto consider = [&](std::vector<GNSSObservation>::const_iterator itr) {
      if (itr == observations_.end()) {
        return;
      }
      const std::size_t index = static_cast<std::size_t>(std::distance(observations_.begin(), itr));
      if (consume_once_ && consumed_[index]) {
        return;
      }
      const double delta = std::abs(itr->stamp - stamp);
      if (delta <= time_tolerance_ && delta < best_delta) {
        best_delta = delta;
        best_index = static_cast<int>(index);
      }
    };

    if (it != observations_.end()) {
      consider(it);
    }
    if (it != observations_.begin()) {
      consider(std::prev(it));
    }
    return best_index;
  }

  void on_smoother_update(gtsam_points::ISAM2Ext& /*isam2*/, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    if (!enabled_ || observations_.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    std::size_t inserted = 0;
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

      const int observation_index = find_nearest_observation(stamp_itr->second);
      if (observation_index < 0) {
        continue;
      }

      const GNSSObservation& obs = observations_[static_cast<std::size_t>(observation_index)];

      gtsam::Vector3 sigma = obs.sigma;
      for (int k = 0; k < 3; ++k) {
        sigma[k] = std::max(sigma[k], min_sigma_);
      }

      new_factors.emplace_shared<gtsam::GPSFactor>(pose_key, obs.position, gtsam::noiseModel::Diagonal::Sigmas(sigma));
      if (consume_once_) {
        consumed_[static_cast<std::size_t>(observation_index)] = true;
      }
      inserted++;
    }

    if (inserted > 0) {
      logger_->info("GNSS prior injection poses={} inserted={}", inserted, inserted);
    }
  }

  bool enabled_ = false;
  double time_tolerance_ = 0.05;
  bool consume_once_ = true;
  double min_sigma_ = 1e-3;
  std::vector<GNSSObservation> observations_;
  std::vector<bool> consumed_;
  std::unordered_map<int, double> submap_stamps_;
  std::mutex mutex_;
  int insert_submap_callback_id_ = -1;
  int smoother_update_callback_id_ = -1;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace glil

extern "C" glil::ExtensionModule* create_extension_module() {
  return new glil::GNSSPriorInjector();
}
