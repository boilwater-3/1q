/**
 * @file scene_transform.h
 * @brief 外部世界运动学 → 平台锚点 ENU 场景状态的一站式转换。
 *
 * 场景目标 ENU 输入契约（AR/EOS/RIR 共用，见 docs/common/contract.md）：
 * 原点 = 当周期平台 ECEF 位置（逐周期重锚）；轴 = 锚点 ENU（x=东/y=北/z=天）；
 * 速度 = 目标 ECEF 速度旋入锚点 ENU 轴（固定锚点旋转，无传输率修正）。
 */

#ifndef ONEQ_COORDINATE_SCENE_TRANSFORM_H_
#define ONEQ_COORDINATE_SCENE_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace coordinate {

/**
 * @brief 平台锚点 ENU 场景状态（位置 + 速度）。
 */
struct ONEQ_API EnuSceneState {
  EnuPositionM position_enu_m{};    /**< 相对锚点的 ENU 位置（m）。 */
  EnuVelocityMps velocity_enu_mps{}; /**< 锚点 ENU 轴下的速度（m/s）。 */
};

/**
 * @brief 将外部运动学（ECEF/LLA 位置 + ECEF 速度）一站式转换为平台锚点 ENU 场景状态。
 * @param[in] kinematics 外部运动学；仅读取与 position_frame 匹配的位置字段，速度固定 ECEF。
 * @param[in] anchor_lla ENU 锚点（平台 ECEF 位置的 LLA 形式；先以 TryEcefToLla 求得）。
 * @param[out] out 输出 ENU 场景状态。
 * @return 成功返回 true；输出为空、输入非有限或坐标变换数值失败返回 false。
 * @note 集成层用法：每周期先用平台 ECEF 求 anchor（一次），再逐目标调用本函数后
 *       直填各模块场景目标输入（ArTargetInput / EosSceneTarget / RirSceneTarget）。
 */
ONEQ_API bool TryMakeEnuSceneState(const ExternalKinematics& kinematics,
                                   const LlaPositionDegM& anchor_lla,
                                   EnuSceneState* out);

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_SCENE_TRANSFORM_H_
