/**
 * @file TargetCategory.h
 * @brief 定义表示当前雷达处理周期内目标的目标分类信息。
 */

#ifndef ONEQ_AIRBORNE_RADAR_MODEL_TARGET_CATEGORY_H_
#define ONEQ_AIRBORNE_RADAR_MODEL_TARGET_CATEGORY_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "1q/api.hpp"

namespace airborne_radar {
namespace model {

/**
 * @brief TargetCategory 封装了单个处理周期的单个目标类别信息。
 * 目前该结构体在数据库匹配后只输出最优匹配类型以及对应该类别的特征值列表，
 * 后续可根据需要扩展更多字段。
 */
struct ONEQ_API TargetCategory {
  std::string target_type{"UNKNOWN"}; /**< 目标类别标识符 */

  float probability{0.0f}; /**< 归一化后的类别概率（来自特征库匹配；启发式分类时为 0） */

  std::unordered_map<std::string, double>
      feature_values; /**< 目标类别的特征值列表，表示该类别的特征集合 */

  TargetCategory() = default; /**< 默认构造函数 */

  explicit TargetCategory(const std::string& type) : target_type(type) {} /**< 带参数的构造函数 */

  /** @brief 添加或更新特征值 */
  void SetFeature(const std::string& feature_name, double value) {
    feature_values[feature_name] = value;
  }
};

/** @brief TargetCategoryList 是 TargetCategory 的列表，表示当前处理周期内所有相关目标类别的特征集合
 */
using TargetCategoryList = std::vector<TargetCategory>;

}  // namespace model
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_MODEL_TARGET_CATEGORY_H_
