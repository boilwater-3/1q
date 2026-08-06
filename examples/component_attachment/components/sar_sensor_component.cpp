/**
 * @file sar_sensor_component.cpp
 * @brief SAR 产品组件实现（孔径积累驱动 + 产品生命周期事件转发）。
 *
 * 每周期从 FlightComponent 读取平台 LLA 位置与航向/速度，航向分解为 NED
 * 速度（北/东分量），姿态零假设（运动学回退与 FD 模式均未暴露姿态，示例
 * 简化，见 README"场景设计"）。点目标真值取自共享场景状态（LLA + RCS）。
 * 图像产品生命周期事件（产出/持续/丢失/失败）由库内 SarProductLifecycle
 * Recorder 承担（Attach 后 StepWithResult 内部自动喂），组件把差分事件
 * 转发为 SarProductEvent（kNoProduct 诊断事件不发布）。
 */

#include "sar_sensor_component.h"

#include <cmath>

#include "core/events.h"
#include "core/world.h"
#include "flight_component.h"
#include "scene_types.h"

namespace component_attachment {

namespace {

/// 产品生命周期事件类型 → 示例事件类型（kNoProduct 诊断事件不转发）。
SarProductEventKind ToDemoKind(sar::session::SarProductLifecycleEventKind kind) {
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

}  // namespace

SarSensorComponent::SarSensorComponent(sar::session::SarSession session)
    : session_(std::move(session)) {
  // 产品生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachProductLifecycleRecorder(&lifecycle_);
}

bool SarSensorComponent::TryApplyRuntimeConfig(
    const sar::config::SarRuntimeConfigPatch& patch) {
  return session_.TryApplyRuntimeConfig(patch);
}

void SarSensorComponent::Step(World& world, double dt_sec) {
  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生产品
  }

  const auto& scene = static_cast<const DemoSceneState&>(world.scene_state());

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
    product.kind = ToDemoKind(event.kind);
    product.stage = event.completed_stage;
    product.estimated_snr_db = event.estimated_snr_db;
    product.abort_reason = event.abort_reason;
    world.signals().on_sar_product(product);
  }
}

}  // namespace component_attachment
