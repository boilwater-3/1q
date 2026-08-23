/**
 * @file runner.h
 * @brief 场景装配与执行入口（RunScene）：全部场景可执行共用的运行体。
 *
 * 场景可执行（scenes/<name>/main.cpp，ONEQ_SCENE_JSON 钉死本场景 JSON）与
 * 通用 runner（app/main.cpp 的 component_attachment_demo，--scene 跑任意
 * 场景）都收敛到 RunScene——装配、周期编排、摘要与冒烟断言只有一份实现。
 */

#ifndef EXAMPLES_APP_RUNNER_H_
#define EXAMPLES_APP_RUNNER_H_

#include <string>

namespace component_attachment {
namespace app {

/// 运行期调试覆盖（命令行 → RunScene；负值/空 = 未指定，用场景文件值）。
struct RunOptions {
  int cycles_override{-1};      /**< < 0 = 未指定，用场景文件 cycles */
  int view_every_override{-1};  /**< < 0 = 未指定，用场景文件 view_log_every_cycles */
  std::string output_dir{};     /**< 空 = 默认（kDefaultOutputDir/<场景名>/） */
};

/**
 * @brief 装配并执行一个场景 JSON。
 *
 * 加载场景（缺省字段静默默认，必填缺失/JSON 语法错误 → 报错退出码 1）→
 * 集成端日志初始化（验收文件钉到输出目录）→ 实体/组件装配（挂载序 =
 * 步进序）→ 周期循环（真值注入 → 指令派发 → World::Step → 落盘）→
 * 结束摘要 + 冒烟断言。返回进程退出码（0 = 冒烟通过）。
 */
int RunScene(const std::string& scene_path, const RunOptions& options);

/**
 * @brief 场景可执行薄入口：解析 --cycles/--view-every/--output-dir 后调
 * RunScene。其余参数（含 --scene）报用法退出——场景可执行的场景由编译宏
 * 钉死，任意场景走通用 runner component_attachment_demo。
 */
int RunSceneWithArgs(const char* scene_json_path, int argc, char* argv[]);

/// 通用 runner 用法打印（--scene/--cycles/--view-every/--output-dir）。
void PrintUsage(const char* program);

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_APP_RUNNER_H_
