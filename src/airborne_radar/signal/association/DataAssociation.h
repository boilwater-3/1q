// Copyright 2026. All Rights Reserved.
//
// Description: Data association based on Mahalanobis distance and LAPJV.

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar {
namespace signal {
namespace association {

struct DataAssociationConfig {
  float speed_sigma{40.0f};
  float rcs_sigma{8.0f};
  float acceleration_sigma{10.0f};
  float unassigned_cost{9.0f};
};

/// @brief DataAssociationEngine keeps lightweight track signatures and
/// associates current detections to previous signatures.
class DataAssociationEngine {
public:
  explicit DataAssociationEngine(DataAssociationConfig config = {});

  /// @brief Associate detections and return stable association keys.
  /// @param targets Current cycle target features.
  /// @param detection_succeeded Detection flags from detection stage.
  /// @return association key per target index (0 means unassociated).
  std::vector<std::uint64_t> Associate(
      const common::TargetFeatureList &targets,
      const std::vector<std::uint8_t> &detection_succeeded);

private:
  struct TrackSignature {
    std::uint64_t key{0};
    Eigen::Vector3f feature{Eigen::Vector3f::Zero()};
  };

  Eigen::Vector3f BuildFeatureVector(const common::TargetFeature &target) const;

  float ComputeMahalanobisSquared(const Eigen::Vector3f &predicted,
                                  const Eigen::Vector3f &measurement) const;

  DataAssociationConfig config_{};
  std::uint64_t next_key_{1};
  std::vector<TrackSignature> previous_tracks_;
};

} // namespace association
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_DATA_ASSOCIATION_H_
