/**
 * @file EsrSceneInput.h
 * @brief 定义 ESR 单周期场景输入聚合类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_INPUT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_INPUT_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/model/EmitterTruthState.h"

namespace electronic_surveillance_radar {
namespace session {

/** @brief EsrSceneEmitter 描述 ESR 场景辐射源输入。 */
using EsrSceneEmitter = model::EmitterTruthState;

/** @brief EsrSceneEmitterList 表示 ESR 场景辐射源输入列表。 */
using EsrSceneEmitterList = model::EmitterTruthStateList;

/**
 * @brief EsrSceneInput 聚合 ESR 单周期场景实体输入。
 */
struct ONEQ_API EsrSceneInput {
  EsrSceneEmitterList emitters{}; /**< 当前周期场景辐射源输入列表 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_INPUT_H_
