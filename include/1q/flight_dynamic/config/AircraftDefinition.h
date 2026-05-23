/**
 * @file AircraftDefinition.h
 * @brief 定义飞行器机型加载参数。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_CONFIG_AIRCRAFT_DEFINITION_H_
#define ONEQ_FLIGHT_DYNAMIC_CONFIG_AIRCRAFT_DEFINITION_H_

#include <string>

#include "1q/api.hpp"

namespace flight_dynamic {
namespace config {

/**
 * @brief 飞行器机型定义，用于指导 JSBSim 加载飞行器模型文件。
 *
 * JSBSim 需要以下文件路径结构（相对于 root_dir）：
 * @code
 * root_dir/
 *   aircraft/<model_name>/<model_name>.xml
 *   engine/
 *   systems/
 * @endcode
 */
struct ONEQ_API AircraftDefinition {
  /**
   * @brief JSBSim 数据文件根目录（绝对路径或相对于工作目录的路径）。
   *
   * 目录下需包含 aircraft/、engine/、systems/ 子目录。
   * 可使用环境变量或部署时确定的路径，空字符串表示使用 JSBSim 默认路径。
   */
  std::string root_dir{};

  /**
   * @brief 飞行器模型名称（不含扩展名），对应 aircraft/<model_name>/ 目录。
   *
   * 示例：`"c172"`, `"f16"`, `"737"`
   */
  std::string model_name{"c172"};
};

}  // namespace config
}  // namespace flight_dynamic

#endif  // ONEQ_FLIGHT_DYNAMIC_CONFIG_AIRCRAFT_DEFINITION_H_
