/**
 * @file RadarSceneInput.h
 * @brief 定义机载雷达单周期场景输入聚合类型。
 */

#ifndef AIRBORNE_RADAR_SESSION_RADAR_SCENE_INPUT_H_
#define AIRBORNE_RADAR_SESSION_RADAR_SCENE_INPUT_H_

#include "1q/airborne_radar/model/TargetFeature.h"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSceneTarget 描述雷达单周期场景目标输入。
 */
using RadarSceneTarget = model::TargetFeature;

/** @brief RadarSceneTargetList 表示雷达场景目标输入列表。 */
using RadarSceneTargetList = model::TargetFeatureList;

/**
 * @brief RadarSceneInput 聚合雷达单周期场景实体输入。
 */
struct RadarSceneInput {
  RadarSceneTargetList targets{}; /**< 当前周期场景目标输入列表 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_SCENE_INPUT_H_
