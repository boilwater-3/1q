/**
 * @file SignalComponentFactory.h
 * @brief 定义 SignalPipeline 私有组件工厂，负责配置映射与组件装配。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/EkfFilter.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/SrifPredictor.h"
#include "airborne_radar/signal/tracking/SrifUpdater.h"
#include "airborne_radar/signal/tracking/SynchronizedTrackPool.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/UdkfPredictor.h"
#include "airborne_radar/signal/tracking/UdkfUpdater.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using ExecutionConfig = ::airborne_radar::config::execution::InternalExecutionConfig;

struct OwnedSignalComponents {
  association::DataAssociationConfig association_config{};
  tracking::TrackFilterConfig track_filter_config{};
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;
  std::unique_ptr<detection::SignalDetector> signal_detector;
};

struct LifecycleAssemblyArtifacts {
  std::unique_ptr<tracking::BoostTrackPool> pool;
  std::unique_ptr<tracking::ITrackPool> pool_wrapper;
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;
  std::vector<std::unique_ptr<tracking::IKalmanPredictor>> imm_predictors_owned;
  std::vector<std::unique_ptr<tracking::IKalmanUpdater>> imm_updaters_owned;
  std::vector<tracking::IKalmanPredictor*> imm_predictors;
  std::vector<tracking::IKalmanUpdater*> imm_updaters;
  std::unique_ptr<tracking::ITrackLifecycleManager> lifecycle_manager;
};

class SignalComponentFactory final {
 public:
  static tracking::LifecycleConfig BuildLifecycleConfig(const ExecutionConfig& config);

  static tracking::TrackFilterConfig BuildTrackFilterConfig(const ExecutionConfig& config);

  static association::DataAssociationConfig BuildAssociationConfig(const ExecutionConfig& config);

  static OwnedSignalComponents BuildOwnedPipelineComponents(const ExecutionConfig& config);

  static LifecycleAssemblyArtifacts BuildLifecycleAssemblyArtifacts(const ExecutionConfig& config);

 private:
  static void LogLifecycleAssemblyConfigViolation(const char* message, float value);
  static void LogLifecycleAssemblyConfigViolation(const char* message, std::size_t value);
  static std::unique_ptr<tracking::IKalmanPredictor> CreateKalmanPredictor(
      float noise_diff_coeff, config::engineering::KalmanUpdateBackend backend);
  static std::unique_ptr<tracking::IKalmanUpdater> CreateKalmanUpdater(
      float measurement_noise_std, config::engineering::KalmanUpdateBackend backend);
  static Eigen::MatrixXf BuildImmTransitionProbability(const ExecutionConfig& config,
                                                       std::size_t model_count);
  static Eigen::VectorXf BuildImmInitialWeights(const ExecutionConfig& config,
                                                std::size_t model_count);
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_
