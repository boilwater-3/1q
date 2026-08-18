/**
 * @file SensorAdapters.h
 * @brief 传感器输出 → 融合探测记录的官方适配器（可选便利层）。
 *
 * 四个传感器模块各自输出自有格式（AR 量测输出帧 / ESR 辐射源假设 / EOS 探测
 * 记录 / SBIRS 探测记录），而 FusionEngine 只接受泛型 fusion::DetectionRecord。
 * 本文件为每种传感器输出提供一个转换函数（默认映射），集成方即开即用；
 * 业务层也可自行适配（DetectionRecord 保持泛型，算法不感知具体类型）。
 *
 * 默认映射固化了示例场景验证过的业务决策：
 * - 跳过非探测记录（ESR hypothesis_id==0 / EOS-SBIRS detected==false）；
 * - 无身份通道（AR 量测/EOS/SBIRS）key=0，走空间/方位相干关联（去真值化纪律）；
 * - 质量归一化基准（10 dB SNR → 1.0、WFOV 门限 4.0 → 1.0）为库默认，
 *   集成方可自选是否覆盖（覆盖 = 自行适配，不修改本文件）。
 */

#ifndef ONEQ_FUSION_SENSOR_ADAPTERS_H_
#define ONEQ_FUSION_SENSOR_ADAPTERS_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArDetectionOutput.h"
#include "1q/airborne_radar/session/ArExternalInputAdapter.h"
#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace fusion {

/** @brief 源通道标识（与 FusionConfig::source_weights 索引一致；索引 0 未用）。 */
constexpr std::uint32_t kArSourceId = 1U;    /**< AR 源通道 */
constexpr std::uint32_t kEsrSourceId = 2U;   /**< ESR 源通道 */
constexpr std::uint32_t kEosSourceId = 3U;   /**< EOS 源通道 */
constexpr std::uint32_t kSbirsSourceId = 4U; /**< SBIRS 源通道 */

/**
 * @brief 把 AR 量测输出帧适配为融合探测记录（key=0 无身份，含位置）。
 * @param[in] source_id 源通道标识（通常传 kArSourceId）。
 * @param[in] platform 生成该量测帧周期的外部平台运动学输入（雷达局部 ENU → ECEF
 *            换算参考；应与生成帧的输入周期一致）。
 * @param[in] frame AR 周期量测输出帧（雷达局部 ENU 位置，TARGET-OQ-1 处置后
 *            传感器公开输出保持量测形态）。
 * @return 探测记录列表；量测无身份键（key=0 走空间/方位门关联，去真值化纪律），
 *         位置局部 ENU→ECEF→LLA（转换失败退化为无位置记录；平台位姿非法时
 *         返回空列表）。质量 = 检测裕量归一化（10 dB margin → 1.0，库默认基准）。
 * @note 量测协方差 R 不进融合通道（DetectionRecord 无协方差字段）：估计层 R
 *       语义沿用 FusionConfig 逐源噪声配置路径（P2 冻结）。
 */
ONEQ_API std::vector<DetectionRecord> AdaptArDetectionsToDetectionRecords(
    std::uint32_t source_id, const airborne_radar::session::ArExternalPoseInput& platform,
    const airborne_radar::session::ArDetectionOutputFrame& frame);

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

}  // namespace fusion

#endif  // ONEQ_FUSION_SENSOR_ADAPTERS_H_
