/**
 * @file RfScene.h
 * @brief 定义工程 RF v2 的世界场景、参数化波形与单程入射链路公共 API。
 * @note 本文件保持 C++11 可用，不生成复数 IQ，也不包含任何传感器探测或受扰判决。
 */

#ifndef ONEQ_ELECTROMAGNETICS_RF_SCENE_H_
#define ONEQ_ELECTROMAGNETICS_RF_SCENE_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace electromagnetics {

/** @brief RF v2 支持的参数化波形类别。 */
enum class RfSceneWaveformKind : std::uint8_t {
  kContinuous = 0,
  kPulseTrain = 1,
  kLinearSweep = 2,
  kBandLimitedNoise = 3,
};

/** @brief RF v2 使用的名义极化类别。 */
enum class RfScenePolarization : std::uint8_t {
  kHorizontal = 0,
  kVertical = 1,
  kRightHandCircular = 2,
  kLeftHandCircular = 3,
  kUnpolarized = 4,
};

/** @brief ECEF 坐标系中的方向向量；求解时会归一化。 */
struct ONEQ_API RfSceneDirection {
  double x{1.0};
  double y{0.0};
  double z{0.0};
};

/** @brief 轴对称远场天线方向图的统计级近似。 */
struct ONEQ_API RfSceneAntennaPattern {
  RfSceneDirection boresight_ecef{};
  double peak_gain_dbi{0.0};
  double half_power_beamwidth_deg{60.0};
  double sidelobe_level_db{-20.0};
  double backlobe_level_db{-40.0};
  double cross_polarization_isolation_db{30.0};
};

/** @brief 发射事实的 platform、equipment 与 emission 唯一身份。 */
struct ONEQ_API RfEmissionIdentity {
  std::uint64_t platform_id{0};
  std::uint64_t equipment_id{0};
  std::uint64_t emission_id{0};
};

/**
 * @brief 一个绝对 world-time 轴上的参数化 RF 活动。
 * @note 调用方应通过 `TryCreateRf*Waveform` 构造；默认值是无效占位值。
 */
struct ONEQ_API RfWaveformSchedule {
  RfSceneWaveformKind kind{RfSceneWaveformKind::kContinuous};
  double activity_start_time_s{0.0};
  double activity_duration_s{0.0};
  double center_frequency_hz{0.0};
  double occupied_bandwidth_hz{0.0};
  double transmit_power_w{0.0};
  double pulse_width_s{0.0};
  double pulse_repetition_interval_s{0.0};
  double first_pulse_time_s{0.0};
  std::uint32_t pulse_count{0};
  double pulse_jitter_fraction{0.0};
  std::uint64_t timing_seed{0};
  std::uint64_t timing_epoch{0};
  double sweep_start_frequency_hz{0.0};
  double sweep_stop_frequency_hz{0.0};
  double sweep_period_s{0.0};
};

/** @brief RF v2 中的一条实际发射事实。 */
struct ONEQ_API RfSceneEmission {
  RfEmissionIdentity identity{};
  coordinate::EcefPositionM position_ecef_m{};
  coordinate::EcefVelocityMps velocity_ecef_mps{};
  RfSceneAntennaPattern antenna{};
  RfScenePolarization polarization{RfScenePolarization::kUnpolarized};
  RfWaveformSchedule waveform{};
};

/** @brief 一个接收世界窗口内由 orchestrator 冻结的 RF 发射场景。 */
struct ONEQ_API RfSceneFrame {
  std::uint64_t world_cycle_index{0};
  double window_start_time_s{0.0};
  double window_duration_s{0.0};
  std::vector<RfSceneEmission> emissions{};
};

/** @brief 同平台发射设备到接收设备的有向硬件隔离路径。 */
struct ONEQ_API RfCoSiteIsolationPath {
  std::uint64_t transmitter_equipment_id{0};
  std::uint64_t receiver_equipment_id{0};
  double isolation_db{0.0};
};

/** @brief 接收设备在一个世界窗口内冻结的硬件与调谐状态。 */
struct ONEQ_API RfSceneReceiverState {
  std::uint64_t platform_id{0};
  std::uint64_t equipment_id{0};
  coordinate::EcefPositionM position_ecef_m{};
  coordinate::EcefVelocityMps velocity_ecef_mps{};
  RfSceneAntennaPattern antenna{};
  RfScenePolarization polarization{RfScenePolarization::kUnpolarized};
  double window_start_time_s{0.0};
  double window_duration_s{0.0};
  double center_frequency_hz{0.0};
  double bandwidth_hz{0.0};
  double receiver_system_loss_db{0.0};
  double minimum_far_field_range_m{1.0};
  std::vector<RfCoSiteIsolationPath> co_site_paths{};
};

/** @brief 调用方提供的单程传播附加损耗。 */
struct ONEQ_API RfIncidentLinkConfig {
  double additional_propagation_loss_db{0.0};
};

/** @brief 一个 emission 到接收设备输入端的统计级入射链路结果。 */
struct ONEQ_API RfIncidentLinkResult {
  RfEmissionIdentity identity{};
  std::uint64_t receiver_platform_id{0};
  std::uint64_t receiver_equipment_id{0};
  bool is_co_site{false};
  double path_length_m{0.0};
  double propagation_delay_s{0.0};
  double doppler_shift_hz{0.0};
  double transmit_antenna_gain_dbi{0.0};
  double receive_antenna_gain_dbi{0.0};
  double free_space_loss_db{0.0};
  double additional_propagation_loss_db{0.0};
  double polarization_mismatch_loss_db{0.0};
  double receiver_system_loss_db{0.0};
  double co_site_isolation_db{0.0};
  double arrival_start_time_s{0.0};
  double arrival_end_time_s{0.0};
  double time_overlap_fraction{0.0};
  double frequency_overlap_fraction{0.0};
  double received_power_before_overlap_w{0.0};
  double received_power_w{0.0};
  double received_power_spectral_density_w_per_hz{0.0};
};

/**
 * @brief 构造连续载波参数化波形。
 * @param[in] start_time_s 绝对 world-time 起点。
 * @param[in] duration_s 活动持续时间。
 * @param[in] center_frequency_hz 中心频率。
 * @param[in] bandwidth_hz 占用带宽。
 * @param[in] transmit_power_w 活动期间的总发射功率。
 * @param[out] waveform 成功时原子写入波形。
 * @return 全部输入合法时返回 true。
 */
ONEQ_API bool TryCreateRfContinuousWaveform(double start_time_s, double duration_s,
                                            double center_frequency_hz, double bandwidth_hz,
                                            double transmit_power_w, RfWaveformSchedule* waveform);

/**
 * @brief 构造带限噪声参数化波形。
 * @param[in] start_time_s 绝对 world-time 起点。
 * @param[in] duration_s 活动持续时间。
 * @param[in] center_frequency_hz 中心频率。
 * @param[in] bandwidth_hz 噪声占用带宽。
 * @param[in] transmit_power_w 活动期间的带内总发射功率。
 * @param[out] waveform 成功时原子写入波形。
 * @return 全部输入合法时返回 true。
 */
ONEQ_API bool TryCreateRfNoiseWaveform(double start_time_s, double duration_s,
                                       double center_frequency_hz, double bandwidth_hz,
                                       double transmit_power_w, RfWaveformSchedule* waveform);

/**
 * @brief 构造具有确定性 jitter 的有限脉冲列。
 * @param[in] first_pulse_time_s 首脉冲绝对 world time。
 * @param[in] center_frequency_hz 中心频率。
 * @param[in] bandwidth_hz 占用带宽。
 * @param[in] peak_power_w 脉冲峰值功率。
 * @param[in] pulse_width_s 脉宽。
 * @param[in] pulse_repetition_interval_s 名义 PRI。
 * @param[in] pulse_count 脉冲数量。
 * @param[in] jitter_fraction PRI 的最大相对抖动，范围 `[0, 0.25]`。
 * @param[in] timing_seed 确定性时序种子。
 * @param[in] timing_epoch 确定性时序 epoch。
 * @param[out] waveform 成功时原子写入波形。
 * @return 全部输入合法且脉冲互不重叠时返回 true。
 */
ONEQ_API bool TryCreateRfPulseTrainWaveform(double first_pulse_time_s, double center_frequency_hz,
                                            double bandwidth_hz, double peak_power_w,
                                            double pulse_width_s,
                                            double pulse_repetition_interval_s,
                                            std::uint32_t pulse_count, double jitter_fraction,
                                            std::uint64_t timing_seed, std::uint64_t timing_epoch,
                                            RfWaveformSchedule* waveform);

/**
 * @brief 构造周期重复的线性扫频波形。
 * @param[in] start_time_s 绝对 world-time 起点。
 * @param[in] duration_s 活动持续时间。
 * @param[in] start_frequency_hz 每次 sweep 的起始中心频率。
 * @param[in] stop_frequency_hz 每次 sweep 的终止中心频率。
 * @param[in] instantaneous_bandwidth_hz 瞬时占用带宽。
 * @param[in] transmit_power_w 活动期间的总发射功率。
 * @param[in] sweep_period_s 一次线性 sweep 的周期。
 * @param[out] waveform 成功时原子写入波形。
 * @return 全部输入合法且起止频率不相等时返回 true。
 */
ONEQ_API bool TryCreateRfLinearSweepWaveform(double start_time_s, double duration_s,
                                             double start_frequency_hz, double stop_frequency_hz,
                                             double instantaneous_bandwidth_hz,
                                             double transmit_power_w, double sweep_period_s,
                                             RfWaveformSchedule* waveform);

/**
 * @brief 校验冻结 RF scene 的身份、窗口、方向图与全部波形。
 * @param[in] scene 待校验场景。
 * @return 场景完整合法且 emission ID 唯一时返回 true。
 */
ONEQ_API bool TryValidateRfSceneFrame(const RfSceneFrame& scene);

/**
 * @brief 求解一个 RF v2 emission 到冻结接收状态的单程入射链路。
 * @param[in] emission 发射事实。
 * @param[in] receiver 接收设备状态。
 * @param[in] config 附加传播损耗。
 * @param[out] result 成功时原子写入结果，失败时保持原值。
 * @return 输入合法且近场或 co-site 路径可求解时返回 true。
 * @note Doppler 约定为闭合时正值，发射远离接收机时为负值。
 */
ONEQ_API bool TryEvaluateRfIncidentLink(const RfSceneEmission& emission,
                                        const RfSceneReceiverState& receiver,
                                        const RfIncidentLinkConfig& config,
                                        RfIncidentLinkResult* result);

/**
 * @brief 按 emission 身份稳定排序并聚合接收功率。
 * @param[in] links 同一接收设备的链路结果。
 * @param[out] total_received_power_w 成功时写入线性功率和。
 * @return 接收设备一致、身份唯一且功率合法时返回 true。
 */
ONEQ_API bool TryAggregateRfIncidentPower(const std::vector<RfIncidentLinkResult>& links,
                                          double* total_received_power_w);

}  // namespace electromagnetics
}  // namespace oneq

#endif  // ONEQ_ELECTROMAGNETICS_RF_SCENE_H_
