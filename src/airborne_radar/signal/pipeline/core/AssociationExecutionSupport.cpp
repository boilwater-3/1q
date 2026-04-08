#include "airborne_radar/signal/pipeline/core/AssociationExecutionSupport.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

void PrepareAssociationSeedsForCycle(
    bool has_manual_association_seeds,
    const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds,
    const tracking::ITrackLifecycleManager* auto_lifecycle_manager,
    association::DataAssociationEngine* association_engine) {
  if (association_engine == nullptr) {
    return;
  }

  if (has_manual_association_seeds) {
    association_engine->SetAssociationSeeds(manual_association_seeds);
    return;
  }
  if (auto_lifecycle_manager != nullptr) {
    association_engine->SetAssociationSeeds(auto_lifecycle_manager->BuildAssociationSeeds());
    return;
  }
  association_engine->ResetAssociationSeedModeToStateless();
}

void RunAssociationPass(const model::TargetFeatureList& input_state,
                        const std::vector<std::uint8_t>& detection_succeeded,
                        const std::vector<tracking::MeasurementCovariance>& measurement_covariances,
                        float dt_sec, association::DataAssociationEngine* association_engine,
                        association::AssociationResult* association_result,
                        std::vector<std::uint64_t>* association_keys) {
  if (association_engine == nullptr || association_result == nullptr ||
      association_keys == nullptr) {
    return;
  }

  *association_result = association_engine->AssociateDetections(input_state, detection_succeeded,
                                                                measurement_covariances, dt_sec);
  *association_keys = association_result->target_keys;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
