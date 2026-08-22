/**
 * @file PolarizationAcceptanceS.h
 * @brief 验收旁路 Sinclair S 派生（不进识别）。
 *
 * 通道：channel_1=HH，channel_2=VV；HV=VH；φ_hv=φ_vh=0。
 * 最近邻样本须同时 has_cross_pol 与 has_phase_vv。
 */

#ifndef ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_POLARIZATION_ACCEPTANCE_S_H_
#define ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_POLARIZATION_ACCEPTANCE_S_H_

#include <vector>

#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace runtime {

/** @brief 验收旁路极化五字段（线性 Span / |det|，角度为度）。 */
struct PolarizationAcceptanceSResult {
  double span{0.0};
  double abs_det{0.0};
  double depolarization{0.0};
  double psi_deg{0.0};
  double tau_deg{0.0};
};

/**
 * @brief 按冻结公式从最近邻样本构造 S 并派生五字段。
 * @return 成功返回 true；缺 has_*、Span=0 或非法输入原子拒绝且不修改 @p result。
 */
bool TryResolvePolarizationAcceptanceS(
    const std::vector<session::RirPolarizationRcsSample>& samples, float look_az_deg,
    float look_el_deg, PolarizationAcceptanceSResult* result);

}  // namespace runtime
}  // namespace remote_identification_radar

#endif
