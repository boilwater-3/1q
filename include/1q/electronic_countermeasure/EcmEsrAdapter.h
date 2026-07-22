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
 * @return 输入估计量均有限且物理字段合法时返回 true；失败时不修改输出。
 */
ONEQ_API bool TryBuildEcmSensorObservationFrame(
    const electronic_surveillance_radar::session::EmitterHypothesisList& hypotheses,
    std::uint32_t source_esr_success_cycle_index, EcmSensorObservationFrame* output);

}  // namespace session
}  // namespace electronic_countermeasure

#endif  // ONEQ_ELECTRONIC_COUNTERMEASURE_ECM_ESR_ADAPTER_H_
