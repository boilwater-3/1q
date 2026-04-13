/**
 * @file EsrCoordinateUtils.h
 * @brief 定义 ESR 对外坐标输入（LLA/ECEF）到库内位姿的轻量适配工具。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_MODEL_ESR_COORDINATE_UTILS_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_MODEL_ESR_COORDINATE_UTILS_H_

#include "1q/api.hpp"
#include "1q/foundation/coordinate_transform.h"
#include "1q/electronic_surveillance_radar/model/EsrOrientationConfig.h"

namespace electronic_surveillance_radar {
namespace model {

/**
 * @brief ESR 局部坐标参考系定义。
 * @note origin_lla 定义 ENU 原点；frame_attitude_deg 定义 ESR 局部坐标相对 ENU 的姿态。
 */
struct ONEQ_API EsrCoordinateReference {
  oneq::foundation::LlaCoordinateDegM origin_lla{};     /**< 参考原点（WGS84 LLA） */
  oneq::foundation::EulerAnglesDeg frame_attitude_deg{}; /**< ESR 局部坐标相对 ENU 的姿态角 */
};

/**
 * @brief 将 ECEF 坐标转换到 ESR 局部坐标。
 * @param[in] position_ecef_m 外部 ECEF 坐标（单位：m）。
 * @param[in] reference ESR 局部坐标参考系。
 * @param[out] position_local_m 输出 ESR 局部坐标（单位：m），可为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryConvertEcefToEsrLocal(const oneq::foundation::EcefCoordinateM& position_ecef_m,
                                       const EsrCoordinateReference& reference,
                                       EsrVector3f* position_local_m);

/**
 * @brief 将 LLA 坐标转换到 ESR 局部坐标。
 * @param[in] position_lla_deg_m 外部 LLA 坐标（单位：deg/m）。
 * @param[in] reference ESR 局部坐标参考系。
 * @param[out] position_local_m 输出 ESR 局部坐标（单位：m），可为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryConvertLlaToEsrLocal(const oneq::foundation::LlaCoordinateDegM& position_lla_deg_m,
                                      const EsrCoordinateReference& reference,
                                      EsrVector3f* position_local_m);

/**
 * @brief 根据 ECEF 坐标构造 ESR 位姿状态。
 * @param[in] position_ecef_m 外部 ECEF 坐标（单位：m）。
 * @param[in] reference ESR 局部坐标参考系。
 * @param[in] velocity_local_mps ESR 局部坐标速度（单位：m/s）。
 * @param[in] attitude_deg 姿态角（单位：deg）。
 * @param[out] pose 输出位姿状态，可为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryMakeEsrPoseFromEcef(const oneq::foundation::EcefCoordinateM& position_ecef_m,
                                     const EsrCoordinateReference& reference,
                                     const EsrVector3f& velocity_local_mps,
                                     const EsrEulerAngleDeg& attitude_deg, EsrPoseState* pose);

/**
 * @brief 根据 LLA 坐标构造 ESR 位姿状态。
 * @param[in] position_lla_deg_m 外部 LLA 坐标（单位：deg/m）。
 * @param[in] reference ESR 局部坐标参考系。
 * @param[in] velocity_local_mps ESR 局部坐标速度（单位：m/s）。
 * @param[in] attitude_deg 姿态角（单位：deg）。
 * @param[out] pose 输出位姿状态，可为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryMakeEsrPoseFromLla(const oneq::foundation::LlaCoordinateDegM& position_lla_deg_m,
                                    const EsrCoordinateReference& reference,
                                    const EsrVector3f& velocity_local_mps,
                                    const EsrEulerAngleDeg& attitude_deg, EsrPoseState* pose);

}  // namespace model
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_MODEL_ESR_COORDINATE_UTILS_H_
