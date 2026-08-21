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
#include "remote_identification_radar/internal/RirRadarEquations.h"
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

  /** @brief 更新运行期上下文（任务域 + 完整策略域）；数据库路径变化时按需加载。 */
  void UpdateRuntime(const config::RirMissionConfig& mission,
                     const config::RirPolicyConfig& policy);

  /** @brief 更新环境域配置（会话初始化或运行期补丁提交后调用）。 */
  void UpdateEnvironment(const config::RirEnvironmentConfig& environment);

  /** @brief 设置静态硬件上下文并重建检测器。 */
  void SetHardware(const config::RirHardwareConfig& hardware);

  /** @brief 设置 RF scene 平台身份（来自 `RirSessionConfig::sensor_platform_id`）。 */
  void SetSensorPlatformId(std::uint64_t sensor_platform_id);

  /**
   * @brief 执行一个自持识别周期。
   * @param[in] input 周期输入（场景目标 + RF 链路）。
   * @param[out] output_frame 识别输出帧（逐内部航迹结论回填）。
   * @param[in] batch_id 本周期会话内部批号（由 RirSession 分配）。
   * @param[in] dwell_center_deg 本周期驻留波束中心（库内驻留调度器给定：
   *            扫描波位或指定识别目标指向；雷达局部 ENU 系，deg）。
   */
  void RunCycle(const session::RirCycleInput& input, session::RirOutputFrame* output_frame,
                std::uint64_t batch_id,
                const config::RirAzimuthElevationDeg& dwell_center_deg =
                    config::RirAzimuthElevationDeg());

  /** @brief 最近周期是否发布了识别效能摘要。 */
  bool HasLatestSummary() const { return has_latest_summary_; }

  /** @brief 最近周期识别效能摘要。 */
  const session::RirRecognitionCycleSummary& GetLatestSummary() const { return latest_summary_; }

  /**
   * @brief 最近周期航迹归属视图（库内键 ↔ 场景真值目标对照）。
   * @note 结果层产品（结果层携带，不进输出帧）；仅已执行周期刷新，
   *       覆盖本周期全部航迹快照（tentative/confirmed/lost）。
   */
  const std::vector<session::RirTrackAttributionRecord>& LatestTrackAttributions() const {
    return last_track_attributions_;
  }

  /** @brief 最近周期实际 RIR 发射帧（`kIdentify` 且 RF 链成功时非空）。 */
  const oneq::electromagnetics::RfEmissionFrame& LatestEmissionFrame() const {
    return last_emission_frame_;
  }

  /**
   * @brief 指定外部目标是否已达识别结论（上一周期口径，供任务生命周期推进）。
   * @param[in] external_target_id 外部目标 ID。
   * @return 该目标任一航迹的识别状态达 kCategoryConfirmed/kModelConfirmed 时为 true。
   * @note 读取上一周期航迹快照（与指向同源滞后一周期，镜像 AR designation 口径）。
   */
  bool IsTargetRecognized(std::uint64_t external_target_id) const;

  /** @brief 当前生效识别特征数据库版本（供 replay 溯源）。 */
  const std::string& ActiveDatabaseVersion() const { return tracker_.ActiveDatabaseVersion(); }

  /** @brief 当前检测随机种子（供 replay 状态溯源）。 */
  std::uint32_t DetectionRandomSeed() const { return detection_random_seed_; }

 private:
  struct RirResolvedRfCycle {
    bool resolved{false};
    bool receiver_saturated{false};
    oneq::electromagnetics::RfEmissionIdentity own_emission_identity{};
    oneq::electromagnetics::RfSceneEmission own_emission{};
    std::vector<oneq::electromagnetics::RfIncidentLinkResult> incident_links{};
    oneq::electromagnetics::RfWaveformSchedule own_transmit_waveform{};
    float carrier_hz{0.0f};
  };

  /** @brief 检测器配置装配。 */
  dwell::RirDetectorConfig MakeDetectorConfig() const;

  /** @brief 解析本周期 RF 前端（自发射 + 可选外部 scene）。 */
  RirResolvedRfCycle ResolveRfCycle(const session::RirCycleInput& input,
                                    const config::RirAzimuthElevationDeg& dwell_center_deg) const;

  /** @brief 环境事实解析：未启用环境效应时退化到阶段 1 旧口径。 */
  void ResolveEnvironment(float* propagation_loss_db, float* clutter_power_w) const;

  /**
   * @brief 逐目标大气物理附加损耗（common 大气单源，与 AR 同口径）。
   * @note 大气物理关闭时返回 0；k 因子经 UpdateEnvironment 预派生（非 4/3 硬值）。
   */
  float ComputeTargetAtmosphericLossDb(float carrier_hz, float platform_altitude_m,
                                       float look_el_deg, bool has_look_angles,
                                       float slant_range_m, float target_position_z_m) const;

  /** @brief 单个目标检测与量测构造；未通过门控返回空 optional 语义（out=false）。 */
  bool TryBuildMeasurement(const session::RirSceneTarget& target, std::size_t source_index,
                           float platform_altitude_m, float propagation_loss_db,
                           float clutter_power_w, const session::RirCycleInput& input,
                           const config::RirAzimuthElevationDeg& dwell_center_deg,
                           const RirResolvedRfCycle& rf_cycle,
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
  config::RirMissionConfig mission_{};
  config::RirPolicyConfig policy_{};
  config::RirEnvironmentConfig environment_{};
  // 大气 k 因子（UpdateEnvironment 时按气象观测派生，缺省 4/3，与 AR 冻结口径一致）。
  float effective_k_factor_{4.0f / 3.0f};
  std::uint64_t sensor_platform_id_{1U};
  config::RirWorkMode work_mode_{config::RirWorkMode::kStby};
  bool recognition_mode_active_{false};
  float sim_time_sec_{0.0f};
  std::uint32_t detection_random_seed_{42U};

  std::unique_ptr<dwell::RirSignalDetector> detector_{};
  tracking::RirTrackAssociator associator_{};
  // 池化生命周期管理器不可拷贝，经 unique_ptr 持有以保持控制器可移动。
  std::unique_ptr<tracking::RirTrackLifecycle> lifecycle_{new tracking::RirTrackLifecycle()};
  internal::RirPropagationModel propagation_model_{};
  std::mt19937 measurement_rng_{42U};

  std::unique_ptr<recognition::RirFeatureDatabase> database_{};
  std::string database_path_{};
  recognition::RirTracker tracker_{};

  session::RirRecognitionCycleSummary latest_summary_{};
  bool has_latest_summary_{false};
  std::vector<tracking::RirTrackState> last_track_snapshots_{}; /**< 上一周期航迹快照（任务判定用）。 */
  std::vector<session::RirTrackAttributionRecord> last_track_attributions_{}; /**< 最近周期归属视图。 */
  oneq::electromagnetics::RfEmissionFrame last_emission_frame_{}; /**< 最近周期实际发射。 */
};

}  // namespace runtime
}  // namespace remote_identification_radar

#endif  // REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_CONTROLLER_H_
