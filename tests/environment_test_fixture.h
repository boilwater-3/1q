// Copyright 2026. All Rights Reserved.
//
// @file environment_test_fixture.h
// @brief 定义环境层测试使用的内存场景构造器与周期辅助函数。

#ifndef TESTS_ENVIRONMENT_TEST_FIXTURE_H_
#define TESTS_ENVIRONMENT_TEST_FIXTURE_H_

#include <utility>

#include "airborne_radar/environment/EnvironmentService.h"

namespace airborne_radar {
namespace tests {
namespace environment_test {

/// @brief 构造默认的环境周期上下文。
/// @param cycle_index 周期号。
/// @param dt_sec 周期步长（单位：s）。
/// @return 可直接传入环境服务的周期上下文。
inline environment::EnvironmentCycleContext MakeEnvironmentCycle(std::uint32_t cycle_index,
                                                                 float dt_sec = 1.0f) {
  environment::EnvironmentCycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.dt_sec = dt_sec;
  return cycle;
}

/// @brief 构造默认的场景干扰源输入。
/// @param technique 干扰技术类型。
/// @param power_db 干扰功率估计（单位：dB）。
/// @return 带默认置信度和零化其他字段的干扰源状态。
inline environment::JammerEmitterState MakeJammerEmitter(environment::JammingTechnique technique,
                                                         float power_db) {
  environment::JammerEmitterState emitter;
  emitter.technique = technique;
  emitter.power_db = power_db;
  emitter.confidence = 1.0f;
  return emitter;
}

/// @brief MemorySceneBuilder 用于在测试中快速构造内存场景。
class MemorySceneBuilder {
 public:
  /// @brief 使用环境默认值初始化场景。
  MemorySceneBuilder() = default;

  /// @brief 设置传播组合项。
  /// @param base_loss_db 基础传播损耗。
  /// @param atmospheric_loss_db 大气附加衰减。
  /// @param terrain_loss_db 地形/多径附加项。
  /// @return 构造器自身，便于链式调用。
  MemorySceneBuilder& WithPropagation(float base_loss_db, float atmospheric_loss_db,
                                      float terrain_loss_db) {
    scene_state_.base_propagation_loss_db = base_loss_db;
    scene_state_.atmospheric_attenuation_db = atmospheric_loss_db;
    scene_state_.terrain_reflection_db = terrain_loss_db;
    return *this;
  }

  /// @brief 设置杂波功率。
  /// @param clutter_power_db 杂波功率（单位：dB）。
  /// @return 构造器自身，便于链式调用。
  MemorySceneBuilder& WithClutter(float clutter_power_db) {
    scene_state_.clutter_power_db = clutter_power_db;
    return *this;
  }

  /// @brief 追加一个场景干扰源。
  /// @param emitter 干扰源输入状态。
  /// @return 构造器自身，便于链式调用。
  MemorySceneBuilder& AddJammer(environment::JammerEmitterState emitter) {
    scene_state_.jammer_emitters.push_back(std::move(emitter));
    return *this;
  }

  /// @brief 生成当前构造结果对应的场景状态。
  /// @return 可用于更新环境服务的场景状态。
  environment::EnvironmentSceneState BuildSceneState() const { return scene_state_; }

  /// @brief 将当前场景更新到环境服务，并提交到指定周期。
  /// @param environment_service 环境服务实例。
  /// @param cycle_index 生效周期号。
  /// @param dt_sec 周期步长（单位：s）。
  void CommitTo(environment::EnvironmentService* environment_service,
                std::uint32_t cycle_index = 1U, float dt_sec = 1.0f) const {
    if (environment_service == nullptr) {
      return;
    }
    environment_service->UpdateSceneState(scene_state_);
    environment_service->BeginCycle(MakeEnvironmentCycle(cycle_index, dt_sec));
  }

 private:
  environment::EnvironmentSceneState scene_state_{};
};

}  // namespace environment_test
}  // namespace tests
}  // namespace airborne_radar

#endif  // TESTS_ENVIRONMENT_TEST_FIXTURE_H_
