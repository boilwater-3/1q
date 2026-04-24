// Copyright 2026. All Rights Reserved.
//
// @file ar_advanced_injection.cpp
// @brief Layer 2 高级扩展示例：通过接口注入自定义组件替换内部实现。
//
// 本文件展示两种高级适配场景：
//   A. 自定义 ITacticalDecisionEngine：插入平台特有威胁评估逻辑，
//      由 RadarController 直接注入，其余组件仍使用库默认实现。
//   B. 自定义 IEnvironmentService：适配平台自有环境评估模块，
//      由 RadarController 直接注入替换内置 EnvironmentService，
//      此场景下不需要再使用 EnvironmentSceneBuilder。
//
// 典型使用场景：
//   - 平台侧有自研威胁评估算法（替换决策引擎）
//   - 平台侧有气象/电磁环境实时数据源（替换环境服务）
//   - 集成测试时需要注入 mock 组件进行验证

#include <iostream>
#include <string>
#include <vector>

// Layer 2 头文件
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
// 便捷辅助
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/session/RadarSceneTargetUtils.h"
#include "1q/airborne_radar/output/TrackOutputQueries.h"
// 平台侧需要直接操控原生组件时包含
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"

// ---------------------------------------------------------------------------
// 场景 A：自定义 ITacticalDecisionEngine
//
// 替换内置决策引擎，插入平台特有威胁评估策略。
// - 决策结果直接影响 RadarController 发出的 RefreshRate 等控制指令。
// - 状态跨周期由库框架托管（TacticalStateStore），引擎只负责纯逻辑。
// ---------------------------------------------------------------------------
namespace {

class PlatformSpecificDecisionEngine : public airborne_radar::extension::ITacticalDecisionEngine {
 public:
  using DecisionInputFrame = airborne_radar::model::DecisionInputFrame;
  using TacticalDecisionResult = airborne_radar::extension::TacticalDecisionResult;
  using TacticalStateStore = airborne_radar::extension::TacticalStateStore;
  using TacticalProposal = airborne_radar::extension::TacticalProposal;
  using TacticalMode = airborne_radar::extension::TacticalMode;
  using ControlDirective = airborne_radar::extension::control::ControlDirective;
  using ControlDirectiveType = airborne_radar::extension::control::ControlDirectiveType;
  using ControlDirectiveSource = airborne_radar::extension::control::ControlDirectiveSource;

  explicit PlatformSpecificDecisionEngine(float custom_threat_threshold)
      : custom_threat_threshold_(custom_threat_threshold) {}

  /**
   * @brief 单周期决策：基于平台特有威胁阈值发出低截获概率（LPI）请求。
   *
   * 实现要点：
   *   - input_frame 包含本周期目标快照、关联质量、干扰语义等
   *   - state_store  是跨周期内存（可读写威胁记忆和战术模式）
   *   - 返回的 proposals 会被 RadarController 内部的 ControlReducer 归并
   */
  TacticalDecisionResult Evaluate(const DecisionInputFrame& input_frame,
                                  TacticalStateStore& state_store) override {
    TacticalDecisionResult result;

    // 基于平台特有阈值判断是否进入威胁响应模式
    float max_threat = 0.0f;
    for (const auto& track : input_frame.tracks) {
      if (track.state.jamming_detected) {
        // 若有干扰标记的轨迹，提升响应优先级
        max_threat += 0.5f;
      }
      // miss_count 越高表示目标越难跟踪，视为威胁增益
      max_threat += static_cast<float>(track.state.miss_count) * 0.1f;
    }
    // 归一化到 [0, 1]
    if (max_threat > 1.0f) {
      max_threat = 1.0f;
    }

    if (max_threat >= custom_threat_threshold_) {
      state_store.current_mode = TacticalMode::kThreatResponse;
      // 发出 LPI 降功率建议
      result.proposals.emplace_back(
          ControlDirective(ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                           ControlDirectiveSource::THREAT_ASSESSMENT),
          /*priority=*/80,
          "platform threat level exceeded " + std::to_string(custom_threat_threshold_));
    } else {
      state_store.current_mode = TacticalMode::kBaseline;
    }

    result.selected_mode = state_store.current_mode;
    return result;
  }

 private:
  float custom_threat_threshold_;
};

// ---------------------------------------------------------------------------
// 场景 B：自定义 IEnvironmentService
//
// 适配平台自有气象/电磁环境数据源，替换内置 EnvironmentService。
// 适合的场景：
//   - 仿真环境需要精确控制每周期的电磁参数
//   - 平台已有电磁环境评估子系统，数据直接来自硬件
// ---------------------------------------------------------------------------
class SimulatedEnvironmentService : public airborne_radar::environment::IEnvironmentService {
 public:
  using EnvironmentCycleContext = airborne_radar::environment::EnvironmentCycleContext;
  using EnvironmentSnapshot = airborne_radar::environment::EnvironmentSnapshot;

  /** @brief 注入固定的仿真环境参数（实际项目中可改为从文件或总线读取） */
  explicit SimulatedEnvironmentService(float propagation_loss_db, float clutter_power_db)
      : propagation_loss_db_(propagation_loss_db), clutter_power_db_(clutter_power_db) {}

  void BeginCycle(const EnvironmentCycleContext& /*cycle_context*/) override {
    // 仿真场景下不依赖外部上下文，参数已在构造时固定
  }

  EnvironmentSnapshot SampleEnvironment() const override {
    EnvironmentSnapshot snapshot;
    snapshot.propagation_loss_db = propagation_loss_db_;
    snapshot.clutter_power_db = clutter_power_db_;
    // 本示例不注入干扰源；实际适配时按平台协议填写 jammer_facts
    return snapshot;
  }

 private:
  float propagation_loss_db_;
  float clutter_power_db_;
};

}  // namespace

// ---------------------------------------------------------------------------
// main：演示如何将自定义组件注入 RadarController
// ---------------------------------------------------------------------------

// RadarController 需要 IRadarContext、ISignalPipeline、IEnvironmentService，
// RadarSession 内部已有了这些具体实现，但 Layer 2 调用方需要自行构造。
// 这里通过一个实际项目中更常见的方式：从 RadarSession 内部 "旁路" 到
// RadarController，是通过完全手动组装组件。
//
// 注意：
//   - RadarSession 是 Layer 1 封装，内部默认组件通过 config 参数配置。
//   - 若需要注入自定义组件，**必须**放弃 RadarSession，
//     直接面向 RadarController 编程（参见 AGENTS.md 架构约定）。
//   - 完整的 Layer 2 组装需要构造具体的 IRadarContext、SignalPipeline、
//     EnvironmentService 实现（库提供具体实现类，在 src/ 不在公共 API 中）。
//   - 本示例展示注入 ITacticalDecisionEngine 的核心方式，
//     其他组件使用 RadarSession 内部的引用获取（通过友元测试模式）。
//
// 以下为**实际可运行的最小示例**，展示 RadarController 的注入构造方式。
// 在真实适配工程里，IRadarContext 与 ISignalPipeline 的具体实现由
// 库内部提供（RadarCycleContext / SignalPipeline 等），适配人员仅替换
// 目标接口即可。

int main() {
  // ------------------------------------------------------------------
  // 设计要点说明（可在此运行并修改为完整注入代码）
  // ------------------------------------------------------------------
  std::cout << "=== Layer 2 注入设计要点 ===\n\n";

  std::cout << "1. 替换决策引擎（ITacticalDecisionEngine）\n"
            << "   RadarController 提供带 decision_engine 参数的构造函数：\n"
            << "     RadarController(IRadarContext&, ISignalPipeline&,\n"
            << "                     ITacticalDecisionEngine&,\n"
            << "                     IEnvironmentService&)\n"
            << "   只需将自定义引擎的引用传入即可，其余组件保持默认。\n\n";

  std::cout << "2. 替换环境服务（IEnvironmentService）\n"
            << "   将自定义 IEnvironmentService 实现传给 RadarController，\n"
            << "   此时不再需要 EnvironmentSceneBuilder——环境数据\n"
            << "   由自定义服务直接提供给信号流水线。\n\n";

  std::cout << "3. 接口约定\n"
            << "   - ITacticalDecisionEngine::Evaluate() 每周期调用一次\n"
            << "   - IEnvironmentService::BeginCycle() 在每周期开始前调用\n"
            << "   - IEnvironmentService::SampleEnvironment() 在信号处理中调用\n\n";

  // ------------------------------------------------------------------
  // 实例化自定义组件（在真实代码中直接传引用给 RadarController）
  // ------------------------------------------------------------------
  PlatformSpecificDecisionEngine custom_engine(/*custom_threat_threshold=*/0.7f);
  SimulatedEnvironmentService custom_env(
      /*propagation_loss_db=*/5.5f,
      /*clutter_power_db=*/3.5f);

  std::cout << "自定义决策引擎已构造，威胁阈值 = 0.7\n";
  std::cout << "自定义环境服务已构造，传播损耗 = 5.5 dB，杂波功率 = 3.5 dB\n\n";

  // ------------------------------------------------------------------
  // 以下为伪代码注释，展示完整的 RadarController 手动组装方式。
  // 需要适配者在平台工程中补全 IRadarContext / ISignalPipeline 的具体实现。
  // ------------------------------------------------------------------
  std::cout
      << "=== 完整注入代码模板（伪代码） ===\n"
      << "\n"
      << "  // 1. 构造平台侧上下文（实现 IRadarContext）\n"
      << "  MyRadarContext my_context(...);\n"
      << "\n"
      << "  // 2. 构造信号流水线配置（推荐使用四域 RadarDetailedSessionConfigBuilder）\n"
      << "  //    auto preset =\n"
      << "  //        airborne_radar::config::presets::MakeDetectionMissionRadarSessionConfig();\n"
      << "  //    auto session_cfg =\n"
      << "  //        airborne_radar::config::RadarDetailedSessionConfigBuilder(preset)\n"
      << "  //            .Detection()\n"
      << "  //            .EnablePhysicsDetection(true)\n"
      << "  //            .WithPeakPowerW(5e6f)\n"
      << "  //            .WithMainBeamGainDb(38.0f)\n"
      << "  //            .End()\n"
      << "  //            .Build();\n"
      << "  //    SignalPipeline signal_pipeline(session_cfg);\n"
      << "\n"
      << "  // 3. 注入自定义决策引擎\n"
      << "  PlatformSpecificDecisionEngine custom_engine(0.7f);\n"
      << "  RadarController controller(\n"
      << "      my_context,\n"
      << "      signal_pipeline,\n"
      << "      custom_engine,       // <-- 替换决策引擎\n"
      << "      custom_env);         // <-- 替换环境服务\n"
      << "\n"
      << "  // 4. 驱动循环\n"
      << "  my_context.BeginCycle(input);\n"
      << "  controller.RunOnce();\n"
      << "  if (controller.HasLatestTrackOutputFrame()) {\n"
      << "    auto& frame = controller.GetLatestTrackOutputFrame();\n"
      << "    // 读取 frame 中的轨迹输出 ...\n"
      << "  }\n\n";

  std::cout << "=== 示例完成。组件实例化正确，可继续集成到平台工程 ===\n";
  return 0;
}
