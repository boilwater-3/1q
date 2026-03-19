// Copyright 2026. All Rights Reserved.

/**
 * @file Gater.h
 * @brief 定义数据关联阶段使用的波门裁剪接口与阈值实现。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_GATER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_GATER_H_

namespace airborne_radar {
namespace signal {
namespace association {
/**
 * @brief 波门裁剪抽象接口。
 */
class IGater {
public:
/**
 * @brief 析构函数。
 */
  virtual ~IGater() = default;
/**
 * @brief 判断当前关联代价是否允许进入候选集合。
 * @param cost 关联代价。
 * @return 若代价满足波门条件则返回 true。
 */
  virtual bool Accept(float cost) const = 0;
};
/**
 * @brief 基于固定代价阈值的波门器。
 */
class CostThresholdGater final : public IGater {
public:
/**
 * @brief 构造波门器。
 * @param max_cost 允许通过的最大代价。
 */
  explicit CostThresholdGater(float max_cost) : max_cost_(max_cost) {}
/**
 * @brief 判断代价是否不超过阈值。
 * @param cost 关联代价。
 * @return 若代价小于等于阈值则返回 true。
 */
  bool Accept(float cost) const override { return cost <= max_cost_; }

private:
/**
 * @brief 允许通过的最大代价值。
 */
  float max_cost_{0.0f};
};

} // namespace association
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_GATER_H_
