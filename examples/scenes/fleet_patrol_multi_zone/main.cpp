/**
 * @file fleet_patrol_multi_zone/main.cpp
 * @brief 场景 fleet_patrol_multi_zone 可执行薄入口：场景 JSON 由编译宏钉死，共用 RunScene。
 *
 * 调试参数 --cycles/--view-every/--output-dir（见 app/runner.h）；本目录
 * fleet_patrol_multi_zone.json 为场景描述、fleet_patrol_multi_zone.md 为期望表。任意场景通用 runner：
 * component_attachment_demo --scene <path>。
 */
#include "app/runner.h"

#ifndef ONEQ_SCENE_JSON
#error "scene target must define ONEQ_SCENE_JSON (see examples/CMakeLists.txt)"
#endif

int main(int argc, char* argv[]) {
  return component_attachment::app::RunSceneWithArgs(ONEQ_SCENE_JSON, argc, argv);
}
