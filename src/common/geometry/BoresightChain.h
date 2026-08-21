/**
 * @file BoresightChain.h
 * @brief 传感器安装矩阵合成链（公共域）：平台姿态 × 安装角 × 安装失准的参考系旋转合成。
 */

#ifndef COMMON_GEOMETRY_BORESIGHT_CHAIN_H_
#define COMMON_GEOMETRY_BORESIGHT_CHAIN_H_

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/types.h"
#include "common/geometry/GeometryTransform.h"

namespace oneq {
namespace common {
namespace geometry {

/**
 * @brief 安装矩阵合成链（参考系无关的纯几何引擎）。
 * @details 基准组合关系：v_reference = R · v_sensor，其中
 *          R = R_reference_body(姿态) · R_body_sensor(安装角)⁻¹
 *              · R_sensor_misalign(安装失准)⁻¹。
 *          传感器系方位/俯仰定义：单位指向向量 v（传感器系）由 az/el 按
 *          v = (cos(el)·cos(az), cos(el)·sin(az), sin(el)) 构造。
 *          零姿态 + 零安装角 + 零失准下链路为恒等变换。
 * @note 参考系由调用方定义（姿态参数把机体系旋进哪个参考系，链路就在哪个参考系
 *       工作）：SBIRS 取 ECI（卫星姿态 Body->ECI），机载模块可取 ENU（平台姿态
 *       Body->ENU）。稳定方式（随体/惯性稳定）属于模块策略，不在本类承载；
 *       惯性稳定所需的"期望参考系光轴 -> 传感器系指向"反解原语由
 *       SensorPointingForDesiredReferenceLos 提供。
 * @note 安装失准语义：作用于传感器系内（等效安装偏置微扰），常值偏置 + 运行期
 *       一次抽取的常值随机微扰由调用方合成后传入；本类只承载纯几何（参考系
 *       <->传感器系旋转、传感器系 az/el 提取、限位钳制），不含任何随机源/
 *       时间演化状态。
 */
class BoresightChain {
 public:
  /** @brief 空链：单位矩阵（零姿态、零安装角、零失准），供默认构造。 */
  BoresightChain() = default;

  /**
   * @brief 由平台姿态与安装角构造合成链（失准为零）。
   * @param[in] attitude_reference_body_deg 平台姿态欧拉角（Z-Y-X，Body->Reference，deg）
   * @param[in] mount_body_sensor_deg 传感器安装偏置角（Body->Sensor，deg）
   */
  BoresightChain(const EulerAnglesDeg& attitude_reference_body_deg,
                 const EulerAnglesDeg& mount_body_sensor_deg);

  /**
   * @brief 由平台姿态、安装角与安装失准角构造合成链。
   * @details 合成关系：R_sensor_to_reference = R_reference_body · R_body_sensor⁻¹ ·
   *          R_sensor_misalign⁻¹（失准作用于传感器系内，等效安装偏置微扰）。
   * @param[in] attitude_reference_body_deg 平台姿态欧拉角（Z-Y-X，Body->Reference，deg）
   * @param[in] mount_body_sensor_deg 传感器安装偏置角（Body->Sensor，deg）
   * @param[in] misalignment_deg 安装失准角误差（deg，调用方合成的总量）
   */
  BoresightChain(const EulerAnglesDeg& attitude_reference_body_deg,
                 const EulerAnglesDeg& mount_body_sensor_deg,
                 const EulerAnglesDeg& misalignment_deg);

  /** @brief 是否为恒等链（零姿态 + 零安装角 + 零失准）。 */
  bool IsIdentity() const;

  /**
   * @brief 传感器系向量旋转到参考系分量。
   * @param[in] sensor_vector 传感器系向量（不要求单位化）
   * @return 参考系分量（模长不变）
   */
  oneq::coordinate::Vector3d RotateSensorToReference(
      const oneq::coordinate::Vector3d& sensor_vector) const;

  /**
   * @brief 参考系向量旋转到传感器系分量。
   * @param[in] reference_vector 参考系向量（不要求单位化）
   * @return 传感器系分量（模长不变）
   */
  oneq::coordinate::Vector3d RotateReferenceToSensor(
      const oneq::coordinate::Vector3d& reference_vector) const;

  /**
   * @brief 传感器系单位指向向量旋转到参考系单位向量。
   * @param[in] sensor_azimuth_deg 传感器系方位角（单位：deg）
   * @param[in] sensor_elevation_deg 传感器系俯仰角（单位：deg）
   * @return 参考系单位向量
   */
  oneq::coordinate::Vector3d ReferenceLosOfSensorPointing(float sensor_azimuth_deg,
                                                          float sensor_elevation_deg) const;

  /**
   * @brief 参考系向量反解为传感器系 az/el（单位：deg，方位对称域 (-180, 180]）。
   * @param[in] reference_los 参考系向量（零向量时俯仰回退 0）
   * @param[out] azimuth_deg 传感器系方位角（单位：deg）
   * @param[out] elevation_deg 传感器系俯仰角（单位：deg）
   */
  void SensorAzElOfReferenceVector(const oneq::coordinate::Vector3d& reference_los,
                                   float* azimuth_deg, float* elevation_deg) const;

  /**
   * @brief 期望参考系光轴（单位向量）经链路反解为传感器系指向（惯性稳定反解原语）。
   * @param[in] desired_reference_los 期望光轴参考系单位向量
   * @param[out] azimuth_deg 传感器系方位角（单位：deg）
   * @param[out] elevation_deg 传感器系俯仰角（单位：deg）
   */
  void SensorPointingForDesiredReferenceLos(
      const oneq::coordinate::Vector3d& desired_reference_los, float* azimuth_deg,
      float* elevation_deg) const;

  /**
   * @brief 按传感器系扫描限位钳制 az/el（钳制到窗口边界，不折叠方位）。
   * @param[in] limits 传感器系扫描限位
   * @param[in,out] azimuth_deg 方位角（单位：deg，钳制后）
   * @param[in,out] elevation_deg 俯仰角（单位：deg，钳制后）
   */
  static void ClampToScanLimits(const AzimuthElevationLimitsDeg& limits, float* azimuth_deg,
                                float* elevation_deg);

 private:
  oneq::coordinate::RotationMatrix3d sensor_to_reference_{};
  oneq::coordinate::RotationMatrix3d reference_to_sensor_{};
  bool identity_{true};
};

}  // namespace geometry
}  // namespace common
}  // namespace oneq

#endif  // COMMON_GEOMETRY_BORESIGHT_CHAIN_H_
