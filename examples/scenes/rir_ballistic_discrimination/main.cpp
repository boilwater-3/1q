/**
 * @file rir_ballistic_discrimination/main.cpp
 * @brief 场景 rir_ballistic_discrimination 可执行薄入口：场景 JSON 由编译宏钉死，共用 RunScene。
 *
 * 四个弹道目标（弹头/重诱饵/轻诱饵/碎片）各配极化散射字典（当前为合成占位值，
 * 待甲方表格原件替换），验证极化散射矩阵统计特征（五量×均值/标准差，方向角
 * 圆统计）在验收日志中的输出。调试参数 --cycles/--view-every/--output-dir
 * （见 app/runner.h）；任意场景通用 runner：component_attachment_demo --scene <path>。
 */
#include "app/runner.h"

#ifndef ONEQ_SCENE_JSON
#error "scene target must define ONEQ_SCENE_JSON (see examples/CMakeLists.txt)"
#endif

int main(int argc, char* argv[]) {
  return component_attachment::app::RunSceneWithArgs(ONEQ_SCENE_JSON, argc, argv);
}
