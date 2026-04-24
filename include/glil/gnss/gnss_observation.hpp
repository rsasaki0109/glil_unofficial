// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <gtsam/base/Vector.h>

namespace glil {

/**
 * Loose-coupled GNSS position observation in the local navigation frame.
 * Latitude/longitude are expected to be pre-projected to a local XYZ frame
 * (e.g. ENU) before being written to CSV. Sigma is per-axis standard
 * deviation in meters.
 */
struct GNSSObservation {
  double stamp = 0.0;                                        ///< sensor timestamp in seconds
  gtsam::Vector3 position = gtsam::Vector3::Zero();          ///< 3D position in local nav frame (meters)
  gtsam::Vector3 sigma = gtsam::Vector3::Ones();             ///< per-axis standard deviation (meters)

  /** @return true when stamp, position, and sigma are finite and sigmas are positive. */
  bool valid() const {
    if (!std::isfinite(stamp) || !position.allFinite() || !sigma.allFinite()) {
      return false;
    }
    for (int i = 0; i < 3; ++i) {
      if (sigma[i] <= 0.0) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace glil
