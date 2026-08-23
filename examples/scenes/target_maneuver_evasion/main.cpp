/**
 * @file target_maneuver_evasion/main.cpp
 * @brief 场景 target_maneuver_evasion 可执行薄入口：场景 JSON 由编译宏钉死，共用 RunScene。
 *
 * 调试参数 --cycles/--view-every/--output-dir（见 app/runner.h）；本目录
 * target_maneuver_evasion.json 为场景描述、target_maneuver_evasion.md 为期望表。任意场景通用 runner：
 * component_attachment_demo --scene <path>。
 */
#include "app/runner.h"

#ifndef ONEQ_SCENE_JSON
#error "scene target must define ONEQ_SCENE_JSON (see examples/CMakeLists.txt)"
#endif

int main(int argc, char* argv[]) {
  return component_attachment::app::RunSceneWithArgs(ONEQ_SCENE_JSON, argc, argv);
}
