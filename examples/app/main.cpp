/**
 * @file app/main.cpp
 * @brief 通用场景 runner（component_attachment_demo）：--scene 跑任意场景 JSON。
 *
 * 参数解析在此，装配与执行见 app/runner.h（RunScene）——每场景可执行
 * （scenes/<name>/main.cpp）与本 runner 共用同一运行体；场景描述加载见
 * scenes/scene_data.h。默认场景 examples/scenes/baseline_takeoff_east/
 * baseline_takeoff_east.json（CA_SCENE_DIR 注入）。
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "app/demo_config.h"
#include "app/runner.h"

namespace app = component_attachment::app;

namespace {

/// 默认场景文件：CMake 注入的场景目录（examples/scenes/）。
constexpr char kDefaultSceneFile[] =
    CA_SCENE_DIR "/baseline_takeoff_east/baseline_takeoff_east.json";

}  // namespace

int main(int argc, char* argv[]) {
  // 命令行参数：--scene / --cycles / --view-every / --output-dir。
  std::string scene_path = kDefaultSceneFile;
  app::RunOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scene") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --scene" << "\n";
        app::PrintUsage(argv[0]);
        return 1;
      }
      scene_path = argv[++i];
    } else if (arg == "--cycles") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --cycles" << "\n";
        app::PrintUsage(argv[0]);
        return 1;
      }
      options.cycles_override = std::atoi(argv[++i]);
    } else if (arg == "--view-every") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --view-every" << "\n";
        app::PrintUsage(argv[0]);
        return 1;
      }
      options.view_every_override = std::atoi(argv[++i]);
    } else if (arg == "--output-dir") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --output-dir" << "\n";
        app::PrintUsage(argv[0]);
        return 1;
      }
      options.output_dir = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      app::PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      app::PrintUsage(argv[0]);
      return 1;
    }
  }
  return app::RunScene(scene_path, options);
}
