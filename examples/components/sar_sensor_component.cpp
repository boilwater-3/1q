/**
 * @file sar_sensor_component.cpp
 * @brief SAR 产品组件实现（孔径积累驱动 + 产品生命周期事件转发）。
 *
 * 1. 从 FlightComponent 读平台 LLA 位置与航向/速度（航向分解为 NED 速度，
 *    姿态零假设，示例简化），点目标真值取自共享场景状态，驱动 SarSession；
 * 2. 产品事件（产出/持续/丢失/失败）由库内 SarProductLifecycleRecorder 差分
 *    产出，组件转发为 SarProductEvent（kNoProduct 诊断事件不发布）；
 * 3. 产品调试视图（阶段型）每周期构建，直写中文人读摘要行到集成端
 *    日志（SAR 无逐目标状态，不适用目标级三模式落盘）。
 */

#include "sar_sensor_component.h"

#include <cmath>

#include "core/events.h"
#include "core/world.h"
#include "logger/logger.h"
#include "logger/logger_i18n.h"
#include "flight_component.h"
#include "core/scene_types.h"

namespace component_attachment {

namespace {

/// 产品生命周期事件类型 → 示例事件类型（kNoProduct 诊断事件不转发）。
SarProductEventKind ToAppKind(sar::session::SarProductLifecycleEventKind kind) {
  switch (kind) {
    case sar::session::SarProductLifecycleEventKind::kImageProduced:
      return SarProductEventKind::kImageProduced;
    case sar::session::SarProductLifecycleEventKind::kProductSustained:
      return SarProductEventKind::kProductSustained;
    case sar::session::SarProductLifecycleEventKind::kProductLost:
      return SarProductEventKind::kProductLost;
    case sar::session::SarProductLifecycleEventKind::kProcessingFailed:
      return SarProductEventKind::kProcessingFailed;
    default:
      break;  // kNoProduct：组件未开启诊断事件，调用方显式跳过，不会到达
  }
  return SarProductEventKind::kProductSustained;
}

/// 示例事件类型 → 中文名（人读日志）。
const char* SarEventKindName(SarProductEventKind kind) {
  switch (kind) {
    case SarProductEventKind::kImageProduced:
      return "产出";
    case SarProductEventKind::kProductSustained:
      return "持续";
    case SarProductEventKind::kProductLost:
      return "丢失";
    case SarProductEventKind::kProcessingFailed:
      return "失败";
  }
  return "未知";
}

/// 处理阶段 → 中文名（人读日志）。
const char* SarStageName(sar::session::SarProcessingStage stage) {
  switch (stage) {
    case sar::session::SarProcessingStage::kNone:
      return "无";
    case sar::session::SarProcessingStage::kRawEcho:
      return "原始回波";
    case sar::session::SarProcessingStage::kL1RdaImage:
      return "L1 RDA 图像";
    case sar::session::SarProcessingStage::kL3BpImage:
      return "L3 BP 图像";
  }
  return "未知";
}

}  // namespace

SarSensorComponent::SarSensorComponent(sar::session::SarSession session)
    : session_(std::move(session)) {
  // 产品生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachProductLifecycleRecorder(&lifecycle_);
}

bool SarSensorComponent::TryApplyRuntimeConfig(
    const sar::config::SarRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return applied;
}

// 阶段型调试视图摘要行（SAR 无逐目标状态，仅单行摘要）。
// 人读行示例：周期=400 执行=否 阶段=无 L1图像=无 L3图像=无 聚焦=无 信噪比=0.0dB
//   目标数=2 问题=[sar.squint_angle_exceeds_limit 孔径斜视角超出配置成像限制。]
void SarSensorComponent::LogDebugView(const sar::session::SarProductDebugView& view) {
  const std::string issues_text = app::FormatIssueText(view.issues);
  // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
  const std::string sar_view_log =
      std::string("周期=") +
      std::to_string(view.input_cycle_index) +
      " 执行=" +
      (view.executed_this_cycle ? "是" : "否") +
      " 阶段=" +
      (SarStageName(view.completed_stage)) +
      " L1图像=" +
      (view.has_l1_image ? "有" : "无") +
      " L3图像=" +
      (view.has_l3_bp_image ? "有" : "无") +
      " 聚焦=" +
      (view.has_focused_pixels ? "有" : "无") +
      " 信噪比=" +
      std::to_string(view.estimated_snr_db) +
      "dB 目标数=" +
      std::to_string(view.point_targets.size()) +
      " 问题=[" +
      (issues_text.empty() ? "无" : issues_text.c_str()) +
      "]";
  CA_LOG_VIEW("sar", "周期={} 执行={} 阶段={} L1图像={} L3图像={} 聚焦={} 信噪比={:.1f}dB 目标数={} 问题=[{}]",
              view.input_cycle_index, view.executed_this_cycle ? "是" : "否",
              SarStageName(view.completed_stage),
              view.has_l1_image ? "有" : "无", view.has_l3_bp_image ? "有" : "无",
              view.has_focused_pixels ? "有" : "无", view.estimated_snr_db,
              view.point_targets.size(), issues_text.empty() ? "无" : issues_text.c_str());
}

void SarSensorComponent::Step(World& world, double dt_sec) {
  if (!powered_on_) {
    last_debug_view_ = sar::session::SarProductDebugView{};  // 关机：调试视图清零（无有效周期）
    LogDebugView(last_debug_view_);
    return;  // 关机：组件不驱动会话（设备不工作，不积累孔径）
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生产品
  }

  const auto& scene = static_cast<const AppSceneState&>(world.scene_state());

  // 平台 LLA + NED 状态：航向/速度分解为 NED 北/东分量，姿态零假设
  // （Roll/Pitch 0、Yaw = 航向，示例简化）。
  sar::session::SarCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.dt_sec = static_cast<float>(dt_sec);
  input.platform.time_s = scene.t_sec;
  input.platform.latitude_deg = flight->position().latitude_deg;
  input.platform.longitude_deg = flight->position().longitude_deg;
  input.platform.altitude_m = flight->position().altitude_m;
  const double heading_rad = flight->heading_deg() * 3.14159265358979323846 / 180.0;
  input.platform.velocity_north_mps = flight->speed_mps() * std::cos(heading_rad);
  input.platform.velocity_east_mps = flight->speed_mps() * std::sin(heading_rad);
  input.platform.velocity_down_mps = 0.0;
  input.platform.roll_deg = 0.0;
  input.platform.pitch_deg = 0.0;
  input.platform.yaw_deg = flight->heading_deg();
  input.point_targets = scene.sar_point_targets;

  const sar::session::SarCycleResult result = session_.StepWithResult(input);
  // 调试视图快照每周期都要构建：它是下方摘要行的数据源（SAR 为阶段型视图，
  // 不适用目标级三密度模式，恒为每周期一行摘要）。
  last_debug_view_ = sar::session::SarProductDebugViewBuilder::Build(input, result);
  LogDebugView(last_debug_view_);
  if (result.status != sar::session::SarCycleStatus::kCompleted) {
    return;  // 周期被拒绝/执行失败：本周期无产品
  }

  // 库内 recorder 已按跨周期状态差分产出本周期事件（产出/持续/丢失/失败）。
  // kNoProduct 诊断事件未开启，显式跳过（防御）。
  for (const auto& event : lifecycle_.GetLastEvents()) {
    if (event.kind == sar::session::SarProductLifecycleEventKind::kNoProduct) {
      continue;  // 诊断事件（未开启 emit_no_product_events）：不发布
    }
    SarProductEvent product;
    product.cycle = scene.cycle;
    product.kind = ToAppKind(event.kind);
    // 事件为集成契约：阶段库枚举展平为镜像枚举（值一致）；日志中文名仍按库枚举映射。
    const sar::session::SarProcessingStage stage = event.completed_stage;
    product.stage = static_cast<EventSarStage>(stage);
    product.estimated_snr_db = event.estimated_snr_db;
    product.abort_reason = event.abort_reason;
    if (product.kind == SarProductEventKind::kProductSustained) {
      // 持续类事件每周期重复：事件模式一下不落盘（信号照常发布）。
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string sar_product_event_log =
          std::string("类型=") +
          (SarEventKindName(product.kind)) +
          " 阶段=" +
          (SarStageName(stage)) +
          " 信噪比=" +
          std::to_string(product.estimated_snr_db) +
          "dB" +
          (product.abort_reason.empty() ? "" : " 中止原因=") +
          (product.abort_reason.c_str());
      CA_LOG_EVENT_DUP(world, "sar_product", "类型={} 阶段={} 信噪比={:.1f}dB{}{}",
                       SarEventKindName(product.kind), SarStageName(stage),
                       product.estimated_snr_db,
                       product.abort_reason.empty() ? "" : " 中止原因=",
                       product.abort_reason.c_str());
    } else {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string sar_product_event_log_2 =
          std::string("类型=") +
          (SarEventKindName(product.kind)) +
          " 阶段=" +
          (SarStageName(stage)) +
          " 信噪比=" +
          std::to_string(product.estimated_snr_db) +
          "dB" +
          (product.abort_reason.empty() ? "" : " 中止原因=") +
          (product.abort_reason.c_str());
      CA_LOG_EVENT(world, "sar_product", "类型={} 阶段={} 信噪比={:.1f}dB{}{}",
                   SarEventKindName(product.kind), SarStageName(stage),
                   product.estimated_snr_db,
                   product.abort_reason.empty() ? "" : " 中止原因=",
                   product.abort_reason.c_str());
    }
    world.signals().on_sar_product(product);
  }
}

}  // namespace component_attachment
