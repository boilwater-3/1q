/**
 * @file RirController.h
 * @brief 远程识别雷达识别链路控制器（内部）。
 *
 * 主链 = 数据库加载（路径变化时按需加载）+ 识别执行（积累/判定/回填/摘要）。
 * 行为为 `ArController` 识别段（审计基线 96de367c）的平移改写：
 * - 工作模式 `RirWorkMode::kIdentify` 对应原 `ArWorkMode::kLrr`；
 * - 波束驻留指向（原 Path A 注入 AR 波束）**不迁移**：独立装备自管波束，
 *   驻留指向调度列为阶段 2 后评估项，本模块当前只消费航迹供给。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_
#define REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"
#include "remote_identification_radar/recognition/RecognitionTracker.h"

namespace remote_identification_radar {
namespace runtime {

/**
 * @brief RirController 负责调度识别链路（积累、判定、输出装配）。
 */
class RirController {
 public:
  RirController() = default;

  /** @brief 更新运行期上下文（工作模式 + 识别策略）；数据库路径变化时按需加载，
   *         加载失败保持原库并记录（识别降级为 kDisabled）。 */
  void UpdateRuntime(config::RirWorkMode work_mode,
                     const config::RirRecognitionPolicy& recognition_config);

  /** @brief 设置静态硬件上下文（发射机/天线/接收机，供效能级 SNR 计算）。 */
  void SetHardware(const config::RirHardwareConfig& hardware) { hardware_ = hardware; }

  /**
   * @brief 执行一个识别周期。
   * @param[in] input 周期输入（场景目标 + 航迹供给）。
   * @param[out] output_frame 识别输出帧（逐航迹结论回填）。
   */
  void RunCycle(const session::RirCycleInput& input, session::RirOutputFrame* output_frame);

  /** @brief 最近周期是否发布了识别效能摘要。 */
  bool HasLatestSummary() const { return has_latest_summary_; }

  /** @brief 最近周期识别效能摘要。 */
  const session::RirRecognitionCycleSummary& GetLatestSummary() const {
    return latest_summary_;
  }

 private:
  config::RirHardwareConfig hardware_{};
  config::RirRecognitionPolicy recognition_config_{};
  config::RirWorkMode work_mode_{config::RirWorkMode::kStby};
  bool recognition_mode_active_{false}; /**< 上一已执行周期是否处于 kIdentify。 */
  float sim_time_sec_{0.0f};            /**< 仿真时间（由调用方输入推进）。 */

  std::unique_ptr<recognition::RirFeatureDatabase> database_{};
  std::string database_path_{};
  recognition::RirTracker tracker_{};

  session::RirRecognitionCycleSummary latest_summary_{};
  bool has_latest_summary_{false};

  /** @brief 效能级 SNR：单站雷达方程回波功率与热噪声底之差（dB）。 */
  float ComputeSnrDb(float rcs_m2, float range_m) const;

  /** @brief 单目标识别观测上下文（视线角、SNR、带宽、驻留）。 */
  recognition::RirObservationContext MakeObservationContext(
      const session::RirSceneTarget& target, float platform_altitude_m) const;
};

}  // namespace runtime
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_
