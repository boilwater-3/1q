// Copyright 2026. All Rights Reserved.
//
// @file TargetCategory.h
// @brief 定义表示当前雷达处理周期内目标的目标分类信息。

#ifndef AIRBORNE_RADAR_COMMON_TARGET_CATEGORY_H_
#define AIRBORNE_RADAR_COMMON_TARGET_CATEGORY_H_

#include <string>
#include <unordered_map>
#include <vector>

namespace airborne_radar {
namespace common {

/// @brief TargetCategory 封装了单个处理周期的单个目标类别信息。
/// 目前该结构体在数据库匹配后只输出最优匹配类型以及对应该类别的特征值列表，后续可根据需要扩展更多字段。
struct TargetCategory {
  /// @brief 目标类别标识符
  std::string target_type{"UNKNOWN"};

  /// @brief 目标类别的特征值列表，表示该类别的特征集合。
  std::unordered_map<std::string, double> feature_values;

  /// @brief 默认构造函数
  TargetCategory() = default;

  /// @brief 带参数的构造函数
  explicit TargetCategory(const std::string &type) : target_type(type) {}

  /// @brief 添加或更新特征值
  void SetFeature(const std::string &feature_name, double value) {
    feature_values[feature_name] = value;
  }
};

/// @brief TargetCategoryList 是 TargetCategory 的列表，表示当前处理周期内所有相关目标类别的特征集合。
using TargetCategoryList = std::vector<TargetCategory>;

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_TARGET_CATEGORY_H_
