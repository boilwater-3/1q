/**
 * @file SbirsPipeline.h
 * @brief SBIRS-inspired WFOV/NFOV pipeline。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_

#include <cstdint>
#include <map>
#include <vector>

#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "sbirs_sensor/config/SbirsInternalExecutionConfig.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/tracking/SbirsTrackingTypes.h"

namespace sbirs_sensor {
namespace pipeline {

/** @brief 目标级状态机 6 状态枚举（design 2.2），驱动 WFOV 发现、NFOV 首次捕获与持续跟踪。 */
enum class SbirsTargetState {
  kUndetected = 0,           /**< 初始或目标未被任何视场发现 */
  kWideCandidate,            /**< WFOV 已发现，等待 NFOV 资源调度 */
  kAwaitingNfovAcquisition,  /**< 已被调度器选为首次捕获目标，本周期执行 NFOV 首次捕获 */
  kTruthAssistedTracking,    /**< 首次捕获成功，进入仿真简化的真值辅助持续跟踪 */
  kEstimatedTracking,        /**< EKF 滤波测量跟踪；由配置开关启用滤波时进入，见 design 2.2/2.5a */
  kLost                      /**< 目标从输入场景消失或传感器关闭 */
};

/**
 * @brief pipeline 运行期状态快照，用于 controller 失败回滚与 replay 复现。
 * @note 包含扫描相位、目标状态表、NFOV 锁定目标、随机源状态与 EKF 滤波状态表。
 */
struct SbirsPipelineSnapshot {
  float scan_azimuth_deg{0.0f};                          /**< 当前 WFOV 扫描方位角，单位 deg */
  std::uint64_t next_detection_id{1U};                   /**< 下一个检测记录 ID */
  std::map<std::uint64_t, SbirsTargetState> target_states{}; /**< 各目标状态表（按 target_id 索引） */
  bool has_locked_target{false};                         /**< 是否有目标锁定 NFOV 资源 */
  std::uint64_t locked_target_id{0U};                    /**< 锁定 NFOV 资源的目标 ID */
  unsigned int random_state{1U};  /**< 误差模型随机源状态（replay 可复现） */
  std::map<std::uint64_t, tracking::SbirsGaussianState> filter_states{}; /**< EKF 滤波状态表（kEstimatedTracking 目标的均值+协方差） */
  std::map<std::uint64_t, unsigned int> nis_gate_exceeded_counts{}; /**< EKF NIS 连续超限计数 */
};

/** @brief 单条 pipeline 内部检测结果，组合原始记录与归属。 */
struct SbirsPipelineDetection {
  output::SbirsDetectionRecord record{};            /**< 原始观测记录 */
  attribution::SbirsDetectionAttributionRecord attribution{}; /**< 仿真归属记录 */
};

/** @brief 单周期 pipeline 执行结果，含扫描相位与本周期检测列表。 */
struct SbirsPipelineResult {
  float scan_azimuth_deg{0.0f};                       /**< 本周期扫描方位角，单位 deg */
  std::vector<SbirsPipelineDetection> detections{};   /**< 检测列表 */
};

/**
 * @brief WFOV/NFOV 双视场探测流水线，执行帧级几何门控、WFOV 发现、状态机决策与 NFOV 捕获/跟踪。
 * @note pipeline 跨周期累积状态（扫描相位、目标状态机、NFOV 锁定、随机源）；
 *       通过 Capture/RestoreRuntimeState 支持 controller 失败回滚与 replay 复现。
 */
class SbirsPipeline {
 public:
  /**
   * @brief 构造 pipeline 并应用初始内部执行配置。
   * @param[in] config 内部执行配置
   */
  explicit SbirsPipeline(const config::SbirsInternalExecutionConfig& config);

  /**
   * @brief 应用新的内部执行配置（runtime patch 立即生效后调用）。
   * @param[in] config 新的内部执行配置
   */
  void ApplyConfig(const config::SbirsInternalExecutionConfig& config);
  /**
   * @brief 执行一个仿真周期的探测流水线。
   * @param[in] input 单周期输入
   * @return 本周期 pipeline 结果
   */
  SbirsPipelineResult RunCycle(const session::SbirsCycleInput& input);

  /** @return 当前运行期状态快照，用于回滚或 replay。 */
  SbirsPipelineSnapshot CaptureRuntimeState() const;
  /**
   * @brief 从快照恢复运行期状态。
   * @param[in] snapshot 待恢复的状态快照
   * @return 恢复成功返回 true，否则返回 false
   */
  bool RestoreRuntimeState(const SbirsPipelineSnapshot& snapshot);

 private:
  config::SbirsInternalExecutionConfig config_{};
  float scan_azimuth_deg_{0.0f};
  std::uint64_t next_detection_id_{1U};
  std::map<std::uint64_t, SbirsTargetState> target_states_{};
  bool has_locked_target_{false};
  std::uint64_t locked_target_id_{0U};
  foundation::SbirsRandomSource random_source_;  // 误差模型确定性随机源
  // EKF 滤波组件（kEstimatedTracking 状态使用；启用 enable_estimated_tracking 时激活）
  tracking::SbirsCvTransitionModel cv_transition_model_{};
  tracking::SbirsAngleMeasurementModel angle_measurement_model_{};
  std::map<std::uint64_t, tracking::SbirsGaussianState> filter_states_{};
  std::map<std::uint64_t, unsigned int> nis_gate_exceeded_counts_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_PIPELINE_H_
