/**
 * @file eos_enu_scene_helpers.h
 * @brief EOS 测试共享辅助：零姿态球坐标（斜距/方位/仰角）→ 平台锚点 ENU 位置。
 *
 * EOS 场景目标输入面为平台锚点 radar-local ENU（docs/common/contract.md ENU 契约）。
 * 测试常以体系球坐标表达目标几何（az = atan2(体系 y, 体系 x)，与库内视线角同约定）；
 * 本辅助在零平台姿态下（体系轴 = ENU 轴，x=东/y=北/z=天）完成球坐标 → ENU 的填充。
 */

#ifndef TESTS_SUPPORT_EOS_ENU_SCENE_HELPERS_H_
#define TESTS_SUPPORT_EOS_ENU_SCENE_HELPERS_H_

#include <cmath>

#include "1q/electro_optical_sensor/session/EosSceneTypes.h"

namespace oneq {
namespace test_support {

/// 零姿态球坐标（range/az/el，deg）→ 平台锚点 ENU 位置并写入目标。
inline void SetEosSphericalLook(electro_optical_sensor::session::EosSceneTarget* target,
                                float range_m, float azimuth_deg, float elevation_deg) {
  const double az_rad = static_cast<double>(azimuth_deg) * 3.14159265358979323846 / 180.0;
  const double el_rad = static_cast<double>(elevation_deg) * 3.14159265358979323846 / 180.0;
  const double horizontal = static_cast<double>(range_m) * std::cos(el_rad);
  target->position_x = static_cast<float>(horizontal * std::cos(az_rad));
  target->position_y = static_cast<float>(horizontal * std::sin(az_rad));
  target->position_z = static_cast<float>(static_cast<double>(range_m) * std::sin(el_rad));
}

}  // namespace test_support
}  // namespace oneq

#endif  // TESTS_SUPPORT_EOS_ENU_SCENE_HELPERS_H_
