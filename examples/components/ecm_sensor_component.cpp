/**
 * @file ecm_sensor_component.cpp
 * @brief ECM 组件实现（ESR 假设驱动 + RF-WORLD 发布）。
 *
 * 每周期读同实体 ESR 组件上一成功周期的去真值化假设（按 batch_id 防重复
 * 消费），调度压制/欺骗干扰，成功执行时将 emission_frame 发布到共享 RF 世界
 * （供 AR/ESR/RIR 消费）；须挂载在 ESR 之后、AR 之前（挂载序 = 步进序）。
 */

#include "ecm_sensor_component.h"

#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "core/world.h"
#include "esr_sensor_component.h"
#include "flight_component.h"
#include "logger/logger.h"
#include "core/rf_world_broker.h"
#include "core/scene_types.h"
#include "components/sensor_utils.h"

namespace component_attachment {

namespace ecm = electronic_countermeasure;

namespace {

const char* EcmCycleStatusName(ecm::session::EcmCycleStatus status) {
  switch (status) {
    case ecm::session::EcmCycleStatus::kExecuted:
      return "已执行";
    case ecm::session::EcmCycleStatus::kSafeStopNoFreshObservation:
      return "无新观测停发";
    case ecm::session::EcmCycleStatus::kPoweredOff:
      return "关机";
    case ecm::session::EcmCycleStatus::kRejectedInvalidInput:
      return "输入拒绝";
    case ecm::session::EcmCycleStatus::kRejectedInvalidConfig:
      return "配置拒绝";
  }
  return "未知";
}

const char* EcmTechniqueName(ecm::EcmTechnique technique) {
  switch (technique) {
    case ecm::EcmTechnique::kSpot:
      return "点频";
    case ecm::EcmTechnique::kBarrage:
      return "阻塞";
    case ecm::EcmTechnique::kSweep:
      return "扫频";
    case ecm::EcmTechnique::kDeception:
      return "欺骗";
  }
  return "未知";
}

}  // namespace

EcmSensorComponent::EcmSensorComponent(ecm::session::EcmSession session)
    : session_(std::move(session)) {}

void EcmSensorComponent::Step(World& world, double dt_sec) {
  if (!powered_on_) {
    return;
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  const EsrSensorComponent* esr = host_ != nullptr ? host_->Find<EsrSensorComponent>() : nullptr;
  if (flight == nullptr || esr == nullptr) {
    return;
  }

  // 共享场景状态（World 只存基类引用，实际类型为 AppSceneState）：读真值组输入，
  // 也向其写回射频发射，故直接取可变引用。
  auto& scene = static_cast<AppSceneState&>(world.scene_state());

  ecm::session::EcmCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.cycle_start_time_s = scene.t_sec;
  input.dt_sec = dt_sec;
  input.input_mode = ecm::EcmInputMode::kSensorDriven;
  input.platform_entity_id = kPlatformEntityId;
  ResolvePlatformEcef(flight->position(), flight->heading_deg(), flight->speed_mps(),
                      &input.platform_position_ecef_m, &input.platform_velocity_ecef_mps);

  if (esr->has_last_completed_output() &&
      esr->last_completed_cycle_index() == static_cast<std::uint32_t>(scene.cycle) &&
      esr->last_batch_id() > last_submitted_esr_batch_id_) {
    ecm::session::EcmSensorObservationFrame sensor_frame;
    if (ecm::session::TryBuildEcmSensorObservationFrame(esr->last_hypotheses(),
                                                        esr->last_batch_id(), &sensor_frame)) {
      input.has_sensor_observation_frame = true;
      input.sensor_observation_frame = sensor_frame;
      last_submitted_esr_batch_id_ = esr->last_batch_id();
    }
  }

  const ecm::session::EcmCycleResult result = session_.StepWithResult(input);
  // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
  const std::string ecm_view_log =
      std::string("周期=") +
      std::to_string(result.input_cycle_index) +
      " 状态=" +
      (EcmCycleStatusName(result.status)) +
      " 发射数=" +
      std::to_string(result.emission_frame.emissions.size()) +
      " ESR批次=" +
      std::to_string(static_cast<unsigned long long>(result.source_esr_batch_id));
  CA_LOG_VIEW("ecm", "周期={} 状态={} 发射数={} ESR批次={}",
              result.input_cycle_index, EcmCycleStatusName(result.status),
              result.emission_frame.emissions.size(),
              static_cast<unsigned long long>(result.source_esr_batch_id));

  if (result.status == ecm::session::EcmCycleStatus::kExecuted) {
    ++executed_cycle_count_;
    if (!result.decisions.empty()) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string ecm_jamming_event_log =
          std::string("技术=") +
          (EcmTechniqueName(result.decisions.front().technique)) +
          " 决策数=" +
          std::to_string(result.decisions.size()) +
          " 功率=" +
          std::to_string(result.decisions.front().allocated_power_w) +
          "W";
      CA_LOG_EVENT(world, "ecm_jamming",
                   "技术={} 决策数={} 功率={:.0f}W",
                   EcmTechniqueName(result.decisions.front().technique),
                   result.decisions.size(), result.decisions.front().allocated_power_w);
    }
    PublishEquipmentEmissions(&scene, result.emission_frame);  // 射频发射 → 共享 RF 世界
  }
}

}  // namespace component_attachment
