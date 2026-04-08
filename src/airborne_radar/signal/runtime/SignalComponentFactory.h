/**
 * @file SignalComponentFactory.h
 * @brief 定义 SignalPipeline 私有组件工厂，负责配置映射与组件装配。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/InternalSignalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
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
namespace runtime {

using pipeline::SignalPipelineConfig;

namespace internal {
/**
 * @brief Pipeline 自持有组件集合。
 */
struct OwnedSignalComponents {
  association::DataAssociationConfig association_config{};     /**< 关联引擎配置。 */
  tracking::TrackFilterConfig track_filter_config{};           /**< 轨迹滤波配置。 */
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor; /**< 可选 Kalman 预测器。 */
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;    /**< 可选 Kalman 更新器。 */
  std::unique_ptr<detection::SignalDetector> signal_detector;  /**< 可选物理探测器。 */
};
/**
 * @brief Lifecycle 自动装配后的组件集合。
 */
struct LifecycleAssemblyArtifacts {
  std::unique_ptr<tracking::BoostTrackPool> pool;              /**< 生命周期对象池。 */
  std::unique_ptr<tracking::ITrackPool> pool_wrapper;          /**< 可选包装的线程安全对象池。 */
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor; /**< 单模型 Kalman 预测器。 */
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;    /**< 单模型 Kalman 更新器。 */
  std::vector<std::unique_ptr<tracking::IKalmanPredictor>>
      imm_predictors_owned; /**< IMM 自持有预测器集合。 */
  std::vector<std::unique_ptr<tracking::IKalmanUpdater>>
      imm_updaters_owned; /**< IMM 自持有更新器集合。 */
  std::vector<const tracking::IKalmanPredictor*>
      imm_predictors; /**< 传递给生命周期管理器的 IMM 预测器视图。 */
  std::vector<const tracking::IKalmanUpdater*>
      imm_updaters; /**< 传递给生命周期管理器的 IMM 更新器视图。 */
  std::unique_ptr<tracking::ITrackLifecycleManager>
      lifecycle_manager; /**< 自动装配后的生命周期管理器。 */
};
/**
 * @brief SignalComponentFactory 统一负责 Signal 层组件装配。
 */
class SignalComponentFactory final {
 public:
  /**
   * @brief 将公共生命周期配置映射为内部 tracking 配置。
   */
  static tracking::LifecycleConfig BuildLifecycleConfig(
      const SignalPipelineConfig& config,
      const pipeline::internal::InternalSignalPipelineConfig& internal_config);
  /**
   * @brief 从顶层配置构造轨迹滤波配置。
   * @param internal_config Signal 顶层配置对应的内部扩展配置。
   * @return TrackFilter 使用的配置。
   */
  static tracking::TrackFilterConfig BuildTrackFilterConfig(
      const pipeline::internal::InternalSignalPipelineConfig& internal_config);
  /**
   * @brief 从顶层配置构造数据关联配置。
   * @param config Signal 顶层配置。
   * @param internal_config Signal 顶层配置对应的内部扩展配置。
   * @return DataAssociationEngine 使用的配置。
   */
  static association::DataAssociationConfig BuildAssociationConfig(
      const SignalPipelineConfig& config,
      const pipeline::internal::InternalSignalPipelineConfig& internal_config);
  /**
   * @brief 构造 Pipeline 自持有组件。
   * @param config Signal 顶层配置。
   * @param internal_config Signal 顶层配置对应的内部扩展配置。
   * @return 组件与其配置集合。
   */
  static OwnedSignalComponents BuildOwnedPipelineComponents(
      const SignalPipelineConfig& config,
      const pipeline::internal::InternalSignalPipelineConfig& internal_config);
  /**
   * @brief 构造 Lifecycle 自动装配组件。
   * @param config Signal 顶层配置。
   * @param internal_config Signal 顶层配置对应的内部扩展配置。
   * @return 生命周期装配结果。
   */
  static LifecycleAssemblyArtifacts BuildLifecycleAssemblyArtifacts(
      const SignalPipelineConfig& config,
      const pipeline::internal::InternalSignalPipelineConfig& internal_config);

 private:
  /**
   * @brief 记录生命周期自动装配配置违规错误日志。
   * @param message 错误消息。
   * @param value 附带数值。
   */
  static void LogLifecycleAssemblyConfigViolation(const char* message, float value);
  /**
   * @brief 记录生命周期自动装配配置违规错误日志。
   * @param message 错误消息。
   * @param value 附带数值。
   */
  static void LogLifecycleAssemblyConfigViolation(const char* message, std::size_t value);
  /**
   * @brief 构造 Kalman 预测器。
   * @param noise_diff_coeff 过程噪声扩散系数。
   * @return 已创建的预测器。
   */
  static std::unique_ptr<tracking::IKalmanPredictor> CreateKalmanPredictor(
      float noise_diff_coeff, config::KalmanUpdateBackend backend);
  /**
   * @brief 构造 Kalman 更新器。
   * @param measurement_noise_std 量测噪声标准差。
   * @return 已创建的更新器。
   */
  static std::unique_ptr<tracking::IKalmanUpdater> CreateKalmanUpdater(
      float measurement_noise_std, config::KalmanUpdateBackend backend);
  /**
   * @brief 构建 IMM 转移矩阵。
   * @param internal_config 内部扩展配置。
   * @param model_count IMM 模型数。
   * @return 转移概率矩阵。
   */
  static Eigen::MatrixXf BuildImmTransitionProbability(
      const pipeline::internal::InternalSignalPipelineConfig& internal_config,
      std::size_t model_count);
  /**
   * @brief 构建 IMM 初始权重。
   * @param internal_config 内部扩展配置。
   * @param model_count IMM 模型数。
   * @return 初始权重向量。
   */
  static Eigen::VectorXf BuildImmInitialWeights(
      const pipeline::internal::InternalSignalPipelineConfig& internal_config,
      std::size_t model_count);
};

}  // namespace internal
}  // namespace runtime
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_SIGNAL_COMPONENT_FACTORY_H_
