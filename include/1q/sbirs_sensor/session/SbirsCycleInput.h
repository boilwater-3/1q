/**
 * @file SbirsCycleInput.h
 * @brief 定义 SBIRS-inspired 单周期输入。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace session {

/**
 * @brief 欧拉角姿态（Z-Y-X 顺序，yaw/pitch/roll，单位 deg，double 精度）。
 * @note 与 oneq::coordinate::EulerAnglesDeg 同约定（正 pitch = 正仰角）；
 *       会话层自有 POD，沿 SbirsVector3M 惯例。
 */
struct ONEQ_API SbirsEulerAnglesDeg {
  double yaw_deg{0.0};   /**< 偏航角（单位：deg） */
  double pitch_deg{0.0}; /**< 俯仰角（单位：deg） */
  double roll_deg{0.0};  /**< 横滚角（单位：deg） */
};

/**
 * @brief 单周期输入，描述一个仿真步的平台几何、目标场景与环境快照。
 * @note 纯数据类型 (POD)。卫星位置用于地球遮挡门控与辐射/噪声几何；卫星速度（必填，
 *       ECEF 零向量合法——如 GEO 卫星）旋入 ECI 后与目标速度合成相对速度，驱动动态
 *       滞后误差、cue 延迟外推与 EKF R 阵；卫星姿态（必填，Body->ECI，零欧拉合法 =
 *       体轴对齐 ECI）与安装角复合为指向合成链，驱动 WFOV/NFOV 内部光轴几何；
 *       `dt_sec` 推进扫描相位。输出 az/el 保持 ECI 极坐标参考，不随姿态/安装变化。
 */
struct ONEQ_API SbirsCycleInput {
  std::uint32_t cycle_index{0U};        /**< 周期序号 */
  float dt_sec{1.0f};                   /**< 本周期步长，单位 s */
  double utc_julian_day{0.0};           /**< UTC 儒略日（JD_UTC，必填且须为有限正数；0 表示未提供）。
                                             ECI 输出参考系需要该时刻的 GMST 完成 ECEF→ECI 旋转 */
  bool has_satellite_position{false};   /**< 是否提供卫星位置 */
  SbirsVector3M satellite_position_ecef_m{}; /**< 卫星 ECEF 位置，单位 m */
  bool has_satellite_velocity_ecef_m_per_s{false}; /**< 是否提供卫星速度（必填：缺失即校验拒绝） */
  SbirsVector3M satellite_velocity_ecef_m_per_s{}; /**< 卫星 ECEF 速度，单位 m/s（零向量合法，如 GEO） */
  bool has_satellite_attitude{false}; /**< 是否提供卫星姿态（必填：缺失即校验拒绝） */
  SbirsEulerAnglesDeg satellite_attitude_eci_body_deg{}; /**< 卫星姿态欧拉角（Z-Y-X，Body->ECI，
                                                              单位 deg；零欧拉合法） */
  SbirsSceneTargetList scene{};         /**< 目标场景列表 */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
