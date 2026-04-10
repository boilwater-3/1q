/**
 * @file EosCoordinateUtils.h
 * @brief 定义 EOS 对外坐标输入（LLA/ECEF）到库内输入载荷的轻量适配工具。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_COORDINATE_UTILS_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_COORDINATE_UTILS_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/common/coordinate_transform.h"
#include "1q/electro_optical_sensor/model/EosCycleInput.h"

namespace electro_optical_sensor {
namespace utils {

/**
 * @brief EOS 局部坐标参考系定义。
 * @note origin_lla 定义 ENU 原点；frame_attitude_deg 定义 EOS 局部坐标相对 ENU 的姿态。
 */
struct ONEQ_API EosCoordinateReference {
  oneq::common::LlaCoordinateDegM origin_lla{};      /**< 参考原点（WGS84 LLA） */
  oneq::common::EulerAnglesDeg frame_attitude_deg{}; /**< EOS 局部坐标相对 ENU 的姿态角 */
};

/**
 * @brief EOS 目标辐射/外观参数集合。
 */
struct ONEQ_API EosTargetAppearance {
  float apparent_temperature_k{290.0f}; /**< 目标等效温度（单位：K） */
  float emissivity{0.9f};               /**< 红外辐射效率，范围 [0, 1] */
  float reflectance{0.2f};              /**< 可见光反射率，范围 [0, 1] */
  float projected_area_m2{1.0f};        /**< 等效投影面积（单位：m^2） */
};

/**
 * @brief EOS 坐标适配执行状态。
 */
enum class ONEQ_API EosCoordinateStatus {
  kOk = 0,                  /**< 转换或构造成功 */
  kNullOutput,              /**< 输出指针为空 */
  kCoordinateTransformFail, /**< 坐标转换失败 */
  kDegenerateGeometry       /**< 几何关系退化（如目标与平台重合） */
};

/**
 * @brief 将 ECEF 坐标转换到 EOS 局部坐标。
 * @param[in] position_ecef_m 外部 ECEF 坐标（单位：m）。
 * @param[in] reference EOS 局部坐标参考系。
 * @param[out] position_local_m 输出 EOS 局部坐标（单位：m），可为 nullptr。
 * @param[out] status 可选状态输出，可为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryConvertEcefToEosLocal(const oneq::common::EcefCoordinateM& position_ecef_m,
                                       const EosCoordinateReference& reference,
                                       oneq::common::Vector3f* position_local_m,
                                       EosCoordinateStatus* status = nullptr);

/**
 * @brief 将 LLA 坐标转换到 EOS 局部坐标。
 * @param[in] position_lla_deg_m 外部 LLA 坐标（单位：deg/m）。
 * @param[in] reference EOS 局部坐标参考系。
 * @param[out] position_local_m 输出 EOS 局部坐标（单位：m），可为 nullptr。
 * @param[out] status 可选状态输出，可为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryConvertLlaToEosLocal(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                                      const EosCoordinateReference& reference,
                                      oneq::common::Vector3f* position_local_m,
                                      EosCoordinateStatus* status = nullptr);

/**
 * @brief 根据 ECEF 坐标构造 EOS 平台位姿。
 * @param[in] position_ecef_m 外部 ECEF 坐标（单位：m）。
 * @param[in] reference EOS 局部坐标参考系。
 * @param[in] velocity_local_mps EOS 局部坐标速度（单位：m/s）。
 * @param[in] attitude_deg 平台姿态角（单位：deg）。
 * @param[out] pose 输出平台位姿，可为 nullptr。
 * @param[out] status 可选状态输出，可为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryMakeEosPoseFromEcef(const oneq::common::EcefCoordinateM& position_ecef_m,
                                     const EosCoordinateReference& reference,
                                     const oneq::common::Vector3f& velocity_local_mps,
                                     const oneq::common::EulerAnglesDeg& attitude_deg,
                                     oneq::common::PoseState* pose,
                                     EosCoordinateStatus* status = nullptr);

/**
 * @brief 根据 LLA 坐标构造 EOS 平台位姿。
 * @param[in] position_lla_deg_m 外部 LLA 坐标（单位：deg/m）。
 * @param[in] reference EOS 局部坐标参考系。
 * @param[in] velocity_local_mps EOS 局部坐标速度（单位：m/s）。
 * @param[in] attitude_deg 平台姿态角（单位：deg）。
 * @param[out] pose 输出平台位姿，可为 nullptr。
 * @param[out] status 可选状态输出，可为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryMakeEosPoseFromLla(const oneq::common::LlaCoordinateDegM& position_lla_deg_m,
                                    const EosCoordinateReference& reference,
                                    const oneq::common::Vector3f& velocity_local_mps,
                                    const oneq::common::EulerAnglesDeg& attitude_deg,
                                    oneq::common::PoseState* pose,
                                    EosCoordinateStatus* status = nullptr);

/**
 * @brief 根据 ECEF 坐标构造 EOS 目标输入（range/az/el）。
 * @param[in] target_id 目标标识。
 * @param[in] target_ecef_m 目标 ECEF 坐标（单位：m）。
 * @param[in] reference EOS 局部坐标参考系。
 * @param[in] platform_pose EOS 平台位姿（局部坐标）。
 * @param[in] appearance 目标外观参数。
 * @param[out] target 输出目标输入，可为 nullptr。
 * @param[out] status 可选状态输出，可为 nullptr。
 * @return 成功返回 true；输入非法、几何退化或输出为空返回 false。
 */
ONEQ_API bool TryMakeEosTargetFromEcef(std::uint64_t target_id,
                                       const oneq::common::EcefCoordinateM& target_ecef_m,
                                       const EosCoordinateReference& reference,
                                       const oneq::common::PoseState& platform_pose,
                                       const EosTargetAppearance& appearance,
                                       session::EosTargetState* target,
                                       EosCoordinateStatus* status = nullptr);

/**
 * @brief 根据 LLA 坐标构造 EOS 目标输入（range/az/el）。
 * @param[in] target_id 目标标识。
 * @param[in] target_lla_deg_m 目标 LLA 坐标（单位：deg/m）。
 * @param[in] reference EOS 局部坐标参考系。
 * @param[in] platform_pose EOS 平台位姿（局部坐标）。
 * @param[in] appearance 目标外观参数。
 * @param[out] target 输出目标输入，可为 nullptr。
 * @param[out] status 可选状态输出，可为 nullptr。
 * @return 成功返回 true；输入非法、几何退化或输出为空返回 false。
 */
ONEQ_API bool TryMakeEosTargetFromLla(std::uint64_t target_id,
                                      const oneq::common::LlaCoordinateDegM& target_lla_deg_m,
                                      const EosCoordinateReference& reference,
                                      const oneq::common::PoseState& platform_pose,
                                      const EosTargetAppearance& appearance,
                                      session::EosTargetState* target,
                                      EosCoordinateStatus* status = nullptr);

}  // namespace utils

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_COORDINATE_UTILS_H_
