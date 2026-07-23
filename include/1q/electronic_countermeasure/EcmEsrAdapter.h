/**
 * @file EcmEsrAdapter.h
 * @brief 定义 ESR 去真值化假设到 ECM 观测帧的公共适配入口。
 */

#ifndef ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_ESR_ADAPTER_H_
#define ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_ESR_ADAPTER_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/electronic_countermeasure/EcmTypes.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"

namespace electronic_countermeasure {
namespace session {

/**
 * @brief 将 ESR 去真值化假设转换为 sensor-driven ECM 观测帧。
 * @param source_esr_batch_id 发布该帧的 ESR 成功批次 batch_id（ESR 只在成功执行周期自增的批次序号），
 *        作为 fresh-frame provenance；必须非 0。取自 EsrOutputFrame::batch_id。
 * @return 输入估计量均有限且物理字段合法、且 batch_id 非 0 时返回 true；失败时不修改输出。
 */
ONEQ_API bool TryBuildEcmSensorObservationFrame(
    const electronic_surveillance_radar::session::EmitterHypothesisList& hypotheses,
    std::uint64_t source_esr_batch_id, EcmSensorObservationFrame* output);

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_ESR_ADAPTER_H_
