// Copyright 2026. All Rights Reserved.
//
// Description: 定义在战术管线中流转的上下文对象。

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_DECISION_CONTEXT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_DECISION_CONTEXT_H_

#include <vector>

#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/TargetCategory.h"

namespace airborne_radar {
namespace core {
namespace context {

/// @brief TargetClassifierView 只对目标分类模块展示的上下文视图。
/// 该结构体包含当前处理周期的轨迹快照，只允许写分类结果。
struct TargetClassifierView {
  /// @brief 输入轨迹快照（只读）。
  const common::DecisionTrackSnapshotList &track_snapshots;

  /// @brief 目标分类结果（可写）。
  common::TargetCategoryList &target_classification_result;

  /// @brief LPI 来源信息（可写）。
  common::LpiSourceInfo &lpi_source_info;

  /// @brief ECCM 来源信息（只读，携带全局干扰状态）。
  const common::EccmSourceInfo &eccm_source_info;
};

/// @brief LpiControllerView 只对 LPI 控制模块展示的上下文视图。
struct LpiControllerView {
  /// @brief LPI 来源信息（只读）。
  const common::LpiSourceInfo &lpi_source_info;

  /// @brief 管线输出指令（可写）。
  std::vector<common::RadarCommand> &decision_commands;
};

/// @brief EccmControllerView 只对 ECCM 控制模块展示的上下文视图。
struct EccmControllerView {
  /// @brief ECCM 来源信息（只读）。
  const common::EccmSourceInfo &eccm_source_info;

  /// @brief 管线输出指令（可写）。
  std::vector<common::RadarCommand> &decision_commands;
};

/// @brief DecisionContext 表示行为决策层的上下文信息，包含当前处理周期的轨迹快照与其他相关信息。
/// 遵循 CQRS 原则，它将系统状态输入（Payload）和战术动作输出（Command Sink）分离。
struct DecisionContext {
  /// @brief 当前处理周期的决策输入帧（只读）。
  const common::DecisionInputFrame input_frame;

  /// @brief 目标分类结果，由 TargetClassifier 写入，后续节点读取。
  common::TargetCategoryList target_classification_result;

  /// @brief LPI 模块的来源信息，由 TargetClassifier 写入。
  common::LpiSourceInfo lpi_source_info;

  /// @brief ECCM 模块的来源信息，由 DecisionContext 在构建时从输入中汇总。
  common::EccmSourceInfo eccm_source_info;

  /// @brief 决策管线输出，存储下发到底层的指令序列。
  std::vector<common::RadarCommand> decision_commands;

  /// @brief 构造函数，向本次处理周期注入输入状态。
  /// @param input 当前周期决策输入帧。
  explicit DecisionContext(const common::DecisionInputFrame &input)
      : input_frame(input),
        target_classification_result(),
        lpi_source_info(),
        eccm_source_info(false),
        decision_commands() {}

  /// @brief 兼容构造：仅使用轨迹快照初始化输入帧。
  explicit DecisionContext(const common::DecisionTrackSnapshotList &snapshots)
      : input_frame(common::DecisionInputFrame(snapshots)),
        target_classification_result(),
        lpi_source_info(),
        eccm_source_info(false),
        decision_commands() {}

  /// @brief 帮助函数：向指令输出箱添加一条指令。
  /// @param cmd 雷达战术指令。
  void AddCommand(common::RadarCommandType cmd_type,
                  common::RadarCommandSource cmd_source,
                  const boost::any &cmd_info = boost::any()) {
    decision_commands.emplace_back(cmd_type, cmd_source, cmd_info);
  }
};

inline TargetClassifierView
CreateTargetClassifierView(DecisionContext &context) {
  return TargetClassifierView{context.input_frame.tracks,
                              context.target_classification_result,
                              context.lpi_source_info,
                              context.eccm_source_info};
}

inline LpiControllerView
CreateLpiControllerView(DecisionContext &context) {
  return LpiControllerView{context.lpi_source_info, context.decision_commands};
}

inline EccmControllerView
CreateEccmControllerView(DecisionContext &context) {
  return EccmControllerView{context.eccm_source_info,
                            context.decision_commands};
}

} // namespace context
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_CONTEXT_DECISION_CONTEXT_H_
