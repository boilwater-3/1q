/**
 * @file RfLinkBudget.h
 * @brief 定义跨模块 RF 发射事实和单程链路预算公共 API。
 */

#ifndef ONEQ_ELECTROMAGNETICS_RF_LINK_BUDGET_H_
#define ONEQ_ELECTROMAGNETICS_RF_LINK_BUDGET_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace electromagnetics {

/** @brief RF 波形的工程级类别，不包含逐采样复数 IQ。 */
enum class RfWaveformKind : std::uint8_t {
  kContinuous = 0,
  kPulsed = 1,
  kSwept = 2,
  kNoise = 3,
};

/** @brief 传感器侧干扰输入的互斥表示模式。 */
enum class RfInterferenceMode : std::uint8_t {
  kNone = 0,
  kLegacy = 1,
  kEngineering = 2,
};

/** @brief RF 发射或接收端的名义极化或极化工作模式。 */
enum class RfPolarization : std::uint8_t {
  kHorizontal = 0,
  kVertical = 1,
  kRightHandCircular = 2,
  kLeftHandCircular = 3,
  kUnpolarized = 4,
  kFullPolarization = 5, /**< 全极化：两条相互垂直的极化通道同时工作。 */
};

/** @brief ECEF 坐标系中的单位方向向量。 */
struct ONEQ_API RfEcefUnitVector {
  double x{1.0};
  double y{0.0};
  double z{0.0};
};

/**
 * @brief 轴对称远场天线方向图的工程近似。
 * @note `boresight_ecef_unit` 在计算时会归一化，但零向量会被拒绝。
 */
struct ONEQ_API RfAntennaPattern {
  RfEcefUnitVector boresight_ecef_unit{};       /**< ECEF 主轴方向 */
  double peak_gain_dbi{0.0};                    /**< 主轴峰值增益（dBi） */
  double half_power_beamwidth_deg{60.0};        /**< 完整 3 dB 波束宽度（deg） */
  double sidelobe_level_db{-20.0};              /**< 相对峰值旁瓣电平（dB） */
  double backlobe_level_db{-40.0};              /**< 相对峰值后瓣电平（dB） */
  double cross_polarization_isolation_db{30.0}; /**< 正交极化隔离（dB） */
};

/** @brief 一个仿真周期内具有固定时频占用和功率的 RF 发射分段。 */
struct ONEQ_API RfEmissionSegment {
  double start_time_s{0.0};        /**< 相对周期起点的开始时间（s） */
  double duration_s{0.0};          /**< 持续时间（s） */
  double center_frequency_hz{0.0}; /**< 中心频率（Hz） */
  double bandwidth_hz{0.0};        /**< 占用带宽（Hz） */
  double transmit_power_w{0.0};    /**< 分段激活期间的总发射功率（W） */
};

/**
 * @brief 发射机发布的 RF 事实。
 * @note 本类型禁止承载接收功率、J/S、受扰判决或成功概率。
 */
struct ONEQ_API RfEmission {
  std::uint64_t emission_id{0};
  std::uint64_t entity_id{0};
  coordinate::EcefPositionM position_ecef_m{};
  coordinate::EcefVelocityMps velocity_ecef_mps{};
  RfAntennaPattern antenna{};
  RfPolarization polarization{RfPolarization::kUnpolarized};
  RfWaveformKind waveform_kind{RfWaveformKind::kContinuous};
  std::vector<RfEmissionSegment> segments{};
};

/** @brief 接收机在当前处理周期内的空间、天线和调谐事实。 */
struct ONEQ_API RfReceiverSite {
  std::uint64_t entity_id{0};
  coordinate::EcefPositionM position_ecef_m{};
  coordinate::EcefVelocityMps velocity_ecef_mps{};
  RfAntennaPattern antenna{};
  RfPolarization polarization{RfPolarization::kUnpolarized};
  double window_start_time_s{0.0};       /**< 接收处理窗口起点（s） */
  double window_duration_s{0.0};         /**< 接收处理窗口长度（s） */
  double center_frequency_hz{0.0};       /**< 调谐中心频率（Hz） */
  double bandwidth_hz{0.0};              /**< 调谐带宽（Hz） */
  double receiver_system_loss_db{0.0};   /**< 接收机前端附加损耗（dB） */
  double minimum_far_field_range_m{1.0}; /**< 远场公式最小适用距离（m） */
  bool has_co_site_isolation{false};
  double co_site_isolation_db{0.0}; /**< 同实体耦合路径隔离（dB） */
};

/** @brief 调用方提供的传播路径附加损耗。 */
struct ONEQ_API RfLinkEvaluationConfig {
  double additional_propagation_loss_db{0.0}; /**< 大气等单程附加损耗（dB） */
};

/** @brief 一个发射分段经过链路和时频窗口后的结果。 */
struct ONEQ_API RfSegmentLinkResult {
  std::size_t segment_index{0};
  double free_space_loss_db{0.0};
  double frequency_overlap_fraction{0.0};
  double time_overlap_fraction{0.0};
  double received_power_before_overlap_w{0.0};
  double received_power_w{0.0};
};

/** @brief 一个发射事实到一个接收站的完整单程链路结果。 */
struct ONEQ_API RfLinkResult {
  std::uint64_t emission_id{0};
  std::uint64_t emitter_entity_id{0};
  std::uint64_t receiver_entity_id{0};
  bool is_co_site{false};
  double path_length_m{0.0};
  double transmit_antenna_gain_dbi{0.0};
  double receive_antenna_gain_dbi{0.0};
  double polarization_mismatch_loss_db{0.0};
  double additional_propagation_loss_db{0.0};
  double receiver_system_loss_db{0.0};
  double co_site_isolation_db{0.0};
  std::vector<RfSegmentLinkResult> segment_results{};
  double total_received_power_w{0.0};
};

/**
 * @brief 计算发射带宽落入接收带宽的比例。
 * @param[in] emission_center_hz 发射中心频率（Hz）。
 * @param[in] emission_bandwidth_hz 发射带宽（Hz）。
 * @param[in] receiver_center_hz 接收中心频率（Hz）。
 * @param[in] receiver_bandwidth_hz 接收带宽（Hz）。
 * @param[out] fraction 成功时写入相对发射总功率的带宽比例；失败时保持原值。
 * @return 输入有限且频率、带宽合法时返回 true。
 */
ONEQ_API bool TryRfFrequencyOverlapFraction(double emission_center_hz, double emission_bandwidth_hz,
                                            double receiver_center_hz, double receiver_bandwidth_hz,
                                            double* fraction);

/**
 * @brief 计算发射分段覆盖接收处理窗口的时间比例。
 * @param[in] emission_start_s 发射开始时间（s）。
 * @param[in] emission_duration_s 发射持续时间（s）。
 * @param[in] receiver_start_s 接收窗口开始时间（s）。
 * @param[in] receiver_duration_s 接收窗口持续时间（s）。
 * @param[out] fraction 成功时写入相对接收处理窗口的覆盖比例；失败时保持原值。
 * @return 输入有限且持续时间为正时返回 true。
 */
ONEQ_API bool TryRfTimeOverlapFraction(double emission_start_s, double emission_duration_s,
                                       double receiver_start_s, double receiver_duration_s,
                                       double* fraction);

/**
 * @brief 校验一个周期内完整 RF 发射帧的事实一致性。
 * @param[in] emissions 当前周期的 RF 发射列表。
 * @param[in] cycle_duration_s 当前周期持续时间（s）。
 * @return 周期时长、全部发射和分段合法且 emission ID 不重复时返回 true。
 */
ONEQ_API bool TryValidateRfEmissionFrame(const std::vector<RfEmission>& emissions,
                                         double cycle_duration_s);

/**
 * @brief 计算一个发射事实到接收站的单程 RF 链路。
 * @param[in] emission 发射事实。
 * @param[in] receiver 接收站事实。
 * @param[in] config 传播附加损耗。
 * @param[out] result 成功时原子写入完整链路结果；失败时保持原值。
 * @return 输入和全部分段合法且链路处于远场或具有同平台隔离时返回 true。
 */
ONEQ_API bool TryEvaluateRfLink(const RfEmission& emission, const RfReceiverSite& receiver,
                                const RfLinkEvaluationConfig& config, RfLinkResult* result);

/**
 * @brief 按发射 ID 确定性聚合多个链路的接收功率。
 * @param[in] links 同一接收实体的链路结果。
 * @param[out] total_received_power_w 成功时写入线性功率和；失败时保持原值。
 * @return 接收实体一致、发射 ID 不重复且功率合法时返回 true。
 */
ONEQ_API bool TryAggregateRfReceivedPower(const std::vector<RfLinkResult>& links,
                                          double* total_received_power_w);

}  // namespace electromagnetics
}  // namespace oneq

#endif  // ONEQ_ELECTROMAGNETICS_RF_LINK_BUDGET_H_
