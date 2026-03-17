// Copyright 2026. All Rights Reserved.
//
// Description: 实现战术责任链中的目标识别处理节点。

#ifndef AIRBORNE_RADAR_DECISION_CLASSIFIER_TARGET_CLASSIFIER_H_
#define AIRBORNE_RADAR_DECISION_CLASSIFIER_TARGET_CLASSIFIER_H_

#include <string>

#include "1q/airborne_radar/decision/pipeline/ITacticalProcessor.h"

namespace airborne_radar {
namespace environment {
namespace database {
class IFeatureRepository;
struct MatchResult;
} // namespace database
} // namespace environment
} // namespace airborne_radar

namespace airborne_radar {
namespace decision {
namespace classifier {

/// @brief TargetClassifier 在管线初期执行计算任务。
/// 处理目标环境的速度或 RCS
/// 以预测目标的危险程度/类型分类，并将缓存传递给管线后续节点。
class TargetClassifier
    : public pipeline::TacticalProcessor<core::context::TargetClassifierView> {
public:
  /// @brief 构造函数，可选注入特征仓储接口。
  explicit TargetClassifier(
      const environment::database::IFeatureRepository *feature_repository =
      nullptr)
    : feature_repository_(feature_repository) {}

protected:
  core::context::TargetClassifierView
  CreateView(core::context::DecisionContext &context) const override;

  /// @brief 执行目标分类算法，仅访问分类模块视图。
  void ProcessView(core::context::TargetClassifierView &view) override;

private:
  /// @brief 识别目标类型。
  /// @param target 输入目标特征。
  /// @return 目标分类标签。
  /// TODO 当前的规则算法仅基于简单加权评分，后续可引入更多特征维度和更复杂的决策逻辑。
  std::string IdentifyTarget(
      const common::DecisionTrackSnapshot &track_snapshot) const;

  /// @brief 根据三维输入计算威胁评分。
  /// @param target 输入目标特征。
  /// @return 计算得到的威胁评分。
  /// TODO 当前的规则算法仅基于简单加权评分，后续可引入更多特征维度和更复杂的决策逻辑。
  float ComputeThreatScore(
      const common::DecisionTrackSnapshot &track_snapshot) const;

  /// @brief 更新供 LPI 使用的来源信息。
  /// @param source_info 待更新的 LPI 来源信息。
  /// @param classification 当前目标的分类标签。
  void UpdateLpiSourceInfo(common::LpiSourceInfo &source_info,
                           const std::string &classification) const;

  /// @brief 判断仓储匹配结果是否足够可靠，可直接采纳。
  bool ShouldAcceptRepositoryMatch(
      const environment::database::MatchResult &match_result) const;

  const environment::database::IFeatureRepository *feature_repository_{nullptr};
};
} // namespace classifier
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_CLASSIFIER_TARGET_CLASSIFIER_H_
