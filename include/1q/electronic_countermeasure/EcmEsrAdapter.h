/**
 * @file EcmEsrAdapter.h
 * @brief 定义 ESR 去真值化假设到 ECM 观测帧的公共适配入口。
 */

#ifndef ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_ESR_ADAPTER_H_
#define ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_ESR_ADAPTER_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_countermeasure/EcmTypes.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"

namespace electronic_countermeasure {
namespace session {

/**
 * @brief 将 ESR 去真值化假设转换为 sensor-driven ECM 观测帧。
 * @param source_esr_batch_id 发布该帧的 ESR 成功批次 batch_id（ESR 只在成功执行周期自增的批次序号），
 *        作为 fresh-frame provenance；必须非 0。取自 EsrOutputFrame::batch_id。
 * @param threat_scores 决策层（threat_assessment）产出的威胁分，按假设索引对齐
 *        （threat_scores[i] 对应 hypotheses[i]；分层契约规则 2：传感器不生产威胁产品，
 *        ECM 以值级消费决策层产品，不引用 threat_assessment 类型）。
 * @return 输入估计量均有限且物理字段合法、batch_id 非 0、threat_scores 与假设数一致
 *         且逐元素有限且 ∈ [0,1] 时返回 true；失败时不修改输出。
 */
ONEQ_API bool TryBuildEcmSensorObservationFrame(
    const electronic_surveillance_radar::session::EmitterHypothesisList& hypotheses,
    std::uint64_t source_esr_batch_id,
    const std::vector<float>& threat_scores, EcmSensorObservationFrame* output);

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_ESR_ADAPTER_H_
