/**
 * @file RirController.h
 * @brief 远程识别雷达自持链路控制器（内部）。
 *
 * 阶段 2-S 起为完整自持链路：
 *   检测（CFAR/6 dB 回退）→ 量测误差 → 门限最近邻关联 → KF/生命周期
 *   → 内部航迹 → 识别积累。
 * 不消费任何 AR 输出；`RirTrackFeed` 公开输入已退役。
 */

#ifndef REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_
#define REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_map>

#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirOutputTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "remote_identification_radar/dwell/RirSignalDetector.h"
#include "remote_identification_radar/internal/RirPropagationModel.h"
#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"
#include "remote_identification_radar/recognition/RecognitionTracker.h"
#include "remote_identification_radar/tracking/RirTrackAssociator.h"
#include "remote_identification_radar/tracking/RirTrackLifecycle.h"

namespace remote_identification_radar {
namespace runtime {

/**
 * @brief RirController 调度自持检测-跟踪-识别链路。
 */
class RirController {
 public:
  RirController() = default;

  /** @brief 更新运行期上下文（工作模式 + 完整策略域）；数据库路径变化时按需加载。 */
  void UpdateRuntime(config::RirWorkMode work_mode, const config::RirPolicyConfig& policy);

  /** @brief 设置静态硬件上下文并重建检测器。 */
  void SetHardware(const config::RirHardwareConfig& hardware);

  /**
   * @brief 执行一个自持识别周期。
   * @param[in] input 周期输入（场景目标 + RF 链路 + 环境快照）。
   * @param[out] output_frame 识别输出帧（逐内部航迹结论回填）。
   */
  void RunCycle(const session::RirCycleInput& input, session::RirOutputFrame* output_frame);

  /** @brief 最近周期是否发布了识别效能摘要。 */
  bool HasLatestSummary() const { return has_latest_summary_; }

  /** @brief 最近周期识别效能摘要。 */
  const session::RirRecognitionCycleSummary& GetLatestSummary() const { return latest_summary_; }

  /** @brief 当前生效识别特征数据库版本（供 replay 溯源）。 */
  const std::string& ActiveDatabaseVersion() const { return tracker_.ActiveDatabaseVersion(); }

  /** @brief 当前检测随机种子（供 replay 状态溯源）。 */
  std::uint32_t DetectionRandomSeed() const { return detection_random_seed_; }

 private:
  /** @brief 检测器配置装配。 */
  dwell::RirDetectorConfig MakeDetectorConfig() const;

  /** @brief 环境事实解析：无环境输入时退化到阶段 1 旧口径。 */
  void ResolveEnvironment(const session::RirCycleInput& input, float* propagation_loss_db,
                          float* clutter_power_w) const;

  /** @brief 单个目标检测与量测构造；未通过门控返回空 optional 语义（out=false）。 */
  bool TryBuildMeasurement(const session::RirSceneTarget& target, std::size_t source_index,
                           float propagation_loss_db, float clutter_power_w,
                           const session::RirCycleInput& input,
                           tracking::RirTrackMeasurement* measurement, float* snr_db);

  /** @brief 计算目标视线角。 */
  static void ComputeLookAngles(const session::RirSceneTarget& target, float* look_az_deg,
                                float* look_el_deg, float* slant_range_m);

  /** @brief 由距离/角度量测误差构造笛卡尔量测协方差。 */
  static tracking::RirMeasurementCovariance MakeCartesianMeasurementCovariance(
      const session::RirSceneTarget& target, float range_std_m, float angle_std_rad);

  /** @brief 在笛卡尔量测协方差下采样量测位置。 */
  tracking::RirTrackMeasurement SampleMeasurementPosition(
      const tracking::RirTrackMeasurement& measurement);

  /** @brief 单目标识别观测上下文（视线角、SNR、带宽、驻留）。 */
  recognition::RirObservationContext MakeObservationContext(const session::RirSceneTarget& target,
                                                            float platform_altitude_m,
                                                            float snr_db) const;

  config::RirHardwareConfig hardware_{};
  config::RirPolicyConfig policy_{};
  config::RirWorkMode work_mode_{config::RirWorkMode::kStby};
  bool recognition_mode_active_{false};
  float sim_time_sec_{0.0f};
  std::uint32_t detection_random_seed_{42U};

  std::unique_ptr<dwell::RirSignalDetector> detector_{};
  tracking::RirTrackAssociator associator_{};
  tracking::RirTrackLifecycle lifecycle_{};
  internal::RirPropagationModel propagation_model_{};
  std::mt19937 measurement_rng_{42U};

  std::unique_ptr<recognition::RirFeatureDatabase> database_{};
  std::string database_path_{};
  recognition::RirTracker tracker_{};

  session::RirRecognitionCycleSummary latest_summary_{};
  bool has_latest_summary_{false};
};

}  // namespace runtime
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_
