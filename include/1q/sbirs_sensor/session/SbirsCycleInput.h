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
 * @brief 单周期输入，描述一个仿真步的平台几何、目标场景与环境快照。
 * @note 纯数据类型 (POD)。卫星位置用于地球遮挡门控与辐射/噪声几何；卫星速度（必填，
 *       ECEF 零向量合法——如 GEO 卫星）旋入 ECI 后与目标速度合成相对速度，驱动动态
 *       滞后误差、cue 延迟外推与 EKF R 阵；`dt_sec` 推进扫描相位。
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
  SbirsSceneTargetList scene{};         /**< 目标场景列表 */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_INPUT_H_
