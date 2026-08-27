/**
 * @file SensorAdapters.h
 * @brief 传感器输出 → 融合探测记录的官方适配器（可选便利层）。
 *
 * 五个传感器通道各自输出自有格式（AR 外部轨迹帧 / ESR 辐射源假设 / EOS 探测
 * 记录 / SBIRS 探测记录 / RIR 特征量测帧），而 FusionEngine 只接受泛型
 * fusion::DetectionRecord。本文件为每种传感器输出提供一个转换函数（默认映射），
 * 集成方即开即用；业务层也可自行适配（DetectionRecord 保持泛型，算法不感知具体类型）。
 *
 * 默认映射固化了示例场景验证过的业务决策：
 * - 跳过非探测记录（AR kLost / ESR hypothesis_id==0 / EOS-SBIRS detected==false /
 *   RIR 全维无效或键 0）；
 * - 无身份通道（EOS/SBIRS）key=0，走方位相干关联（去真值化纪律）；
 * - 质量归一化基准（10 dB SNR → 1.0、WFOV 门限 4.0 → 1.0）为库默认，
 *   集成方可自选是否覆盖（覆盖 = 自行适配，不修改本文件）。
 */

#ifndef ONEQ_FUSION_SENSOR_ADAPTERS_H_
#define ONEQ_FUSION_SENSOR_ADAPTERS_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/remote_identification_radar/session/RirFeatureMeasurementTypes.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace fusion {

/** @brief 源通道标识（与 FusionConfig::source_weights 索引一致；索引 0 未用）。 */
constexpr std::uint32_t kArSourceId = 1U;    /**< AR 源通道 */
constexpr std::uint32_t kEsrSourceId = 2U;   /**< ESR 源通道 */
constexpr std::uint32_t kEosSourceId = 3U;   /**< EOS 源通道 */
constexpr std::uint32_t kSbirsSourceId = 4U; /**< SBIRS 源通道 */
constexpr std::uint32_t kRirSourceId = 5U;   /**< RIR 源通道 */

/**
 * @brief 把 AR 外部轨迹帧适配为融合探测记录（key=关联键，含位置）。
 * @param[in] source_id 源通道标识（通常传 kArSourceId）。
 * @param[in] frame AR 周期外部轨迹输出帧（ECEF 位置）。
 * @return 探测记录列表；跳过 kLost 轨迹（避免以旧位置续命航迹），
 *         位置 ECEF→LLA（转换失败退化为仅身份键记录）。
 */
ONEQ_API std::vector<DetectionRecord> AdaptArTracksToDetectionRecords(
    std::uint32_t source_id,
    const airborne_radar::session::ArExternalTrackOutputFrame& frame);

/**
 * @brief 把 ESR 辐射源假设适配为融合探测记录（key=假设键，方位+射频特征）。
 * @param[in] source_id 源通道标识（通常传 kEsrSourceId）。
 * @param[in] hypotheses ESR 周期辐射源假设列表。
 * @return 探测记录列表；跳过 hypothesis_id==0（库内键 0 = 无身份，不适用
 *         身份直挂），射频特征归一化到可比尺度（GHz/MHz/ms/µs）。
 */
ONEQ_API std::vector<DetectionRecord> AdaptEsrHypothesesToDetectionRecords(
    std::uint32_t source_id,
    const electronic_surveillance_radar::session::EmitterHypothesisList& hypotheses);

/**
 * @brief 把 EOS 探测记录适配为融合探测记录（key=0 无身份，仅方位通道）。
 * @param[in] source_id 源通道标识（通常传 kEosSourceId）。
 * @param[in] records EOS 周期探测记录列表。
 * @return 探测记录列表；跳过未过探测门限记录；质量 = 融合 SNR 归一化
 *         （10 dB → 1.0，库默认基准）。
 */
ONEQ_API std::vector<DetectionRecord> AdaptEosDetectionsToDetectionRecords(
    std::uint32_t source_id,
    const electro_optical_sensor::output::EosDetectionRecordList& records);

/**
 * @brief 把 SBIRS 探测记录适配为融合探测记录（key=0 无身份，仅方位通道，
 *        与 EOS 同构）。
 * @param[in] source_id 源通道标识（通常传 kSbirsSourceId）。
 * @param[in] records SBIRS 周期探测记录列表。
 * @return 探测记录列表；跳过未过探测门限记录；质量 = 线性 IR SNR 相对
 *         WFOV 检测门限归一化（4.0 → 1.0，库默认基准）。
 * @note SBIRS 记录为 ECI 极坐标弧度（2026-08 正式变更）；适配器做 rad→deg
 *       单位换算后填入方位通道（参考系语义由调用方对齐，库内不跨系转换）。
 */
ONEQ_API std::vector<DetectionRecord> AdaptSbirsDetectionsToDetectionRecords(
    std::uint32_t source_id,
    const sbirs_sensor::output::SbirsDetectionRecordList& records);

/**
 * @brief 把 RIR 特征量测帧（双产品出口①）适配为融合探测记录
 *        （key=库内航迹键，方位 + 位置 + 11 维特征 + 可选观测原点）。
 * @param[in] source_id 源通道标识（通常传 kRirSourceId）。
 * @param[in] frame RIR 周期特征量测帧（出口①，调用方从输出帧组装）。
 * @return 探测记录列表；跳过全维无效与键 0 记录；方位做 east→north 参考换算
 *         （RIR 出口① az 自 +x 东起量，融合通道自北，wrap 到 [-180, 180]）；
 *         特征为 11 维固定布局（无效维填 0——NaN 禁止毒化欧氏门，
 *         valid_feature_mask 为权威有效性；0 填失真为 F4 已知近似）；质量 =
 *         有效维质量等权均值（feature_weights 配置口径，缺省等权）；携带
 *         平台位置时 ECEF→LLA 填 sensor_origin（失败退化为无原点记录，AR
 *         先例）；斜距有限且 >0 且原点换算成功时，按自东视线角逆运算还原
 *         东-北-天再 ECEF→LLA 填 has_position（失败维持仅方位+原点）。
 *         位置与方位并存：滤波位置优先，方位留给跨源仅方位关联。
 */
ONEQ_API std::vector<DetectionRecord> AdaptRirFeatureMeasurementsToDetectionRecords(
    std::uint32_t source_id,
    const remote_identification_radar::session::RirFeatureMeasurementFrame& frame);

}  // namespace fusion

#endif  // ONEQ_FUSION_SENSOR_ADAPTERS_H_
