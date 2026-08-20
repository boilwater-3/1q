/**
 * @file eos_enu_scene_helpers.h
 * @brief EOS 测试共享辅助：零姿态球坐标 → ENU，以及平台 ECEF + TryMakeEnuSceneState 手填 CycleInput。
 *
 * EOS 场景目标输入面为平台锚点 radar-local ENU（docs/common/contract.md ENU 契约）。
 * 测试常以体系球坐标表达目标几何（az = atan2(体系 y, 体系 x)，与库内视线角同约定）；
 * 本辅助在零平台姿态下（体系轴 = ENU 轴，x=东/y=北/z=天）完成球坐标 → ENU 的填充。
 * 世界侧 ECEF/LLA 目标请用 TryBuildEosCycleInput / TryFillEosSceneTargetFromKinematics
 * （公共 TryEcefToLla + TryMakeEnuSceneState），不再经已删除的 CycleInputAdapter。
 */

#ifndef TESTS_SUPPORT_EOS_ENU_SCENE_HELPERS_H_
#define TESTS_SUPPORT_EOS_ENU_SCENE_HELPERS_H_

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/scene_transform.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosPlatformEcefPose.h"
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

/// 测试用世界侧目标规格（ECEF/LLA 运动学 + 外观）；手填 CycleInput 前的中间结构。
struct EosWorldTargetSpec {
  std::uint64_t target_id{0U};
  std::string target_name{};
  oneq::coordinate::ExternalKinematics kinematics{};
  electro_optical_sensor::session::EosTargetAppearance appearance{};
};

/// ExternalKinematics + 锚点 LLA → 直填 EosSceneTarget（pos/vel ENU + appearance）。
inline bool TryFillEosSceneTargetFromKinematics(
    std::uint64_t target_id, const oneq::coordinate::ExternalKinematics& kinematics,
    const oneq::coordinate::LlaPositionDegM& anchor_lla,
    const electro_optical_sensor::session::EosTargetAppearance& appearance,
    electro_optical_sensor::session::EosSceneTarget* target,
    const std::string& target_name = std::string()) {
  if (target == nullptr) {
    return false;
  }
  oneq::coordinate::EnuSceneState enu;
  if (!oneq::coordinate::TryMakeEnuSceneState(kinematics, anchor_lla, &enu)) {
    return false;
  }
  target->target_id = target_id;
  target->target_name = target_name;
  target->position_x = static_cast<float>(enu.position_enu_m.east_m);
  target->position_y = static_cast<float>(enu.position_enu_m.north_m);
  target->position_z = static_cast<float>(enu.position_enu_m.up_m);
  target->velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
  target->velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
  target->velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
  target->appearance = appearance;
  return true;
}

/// 平台 ECEF 位姿 + 世界目标规格 → 手填 EosCycleInput（不含 cycle_index，由调用方写入）。
inline bool TryBuildEosCycleInput(
    const electro_optical_sensor::session::EosPlatformEcefPose& platform,
    const std::vector<EosWorldTargetSpec>& targets, float dt_sec,
    electro_optical_sensor::session::EosCycleInput* output) {
  if (output == nullptr) {
    return false;
  }
  oneq::coordinate::LlaPositionDegM anchor_lla;
  if (!oneq::coordinate::TryEcefToLla(platform.platform_position_ecef_m, &anchor_lla)) {
    return false;
  }
  output->dt_sec = dt_sec;
  output->platform_altitude_m = static_cast<float>(anchor_lla.altitude_m);
  output->platform_attitude_deg = platform.platform_attitude_deg;
  output->scene.clear();
  output->scene.reserve(targets.size());
  for (std::size_t i = 0; i < targets.size(); ++i) {
    const EosWorldTargetSpec& spec = targets[i];
    const std::uint64_t id =
        (spec.target_id != 0U) ? spec.target_id : static_cast<std::uint64_t>(i);
    electro_optical_sensor::session::EosSceneTarget scene_target;
    if (!TryFillEosSceneTargetFromKinematics(id, spec.kinematics, anchor_lla, spec.appearance,
                                             &scene_target, spec.target_name)) {
      return false;
    }
    output->scene.push_back(scene_target);
  }
  return true;
}

}  // namespace test_support
}  // namespace oneq

#endif  // TESTS_SUPPORT_EOS_ENU_SCENE_HELPERS_H_
