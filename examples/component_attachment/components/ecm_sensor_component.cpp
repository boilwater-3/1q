/**
 * @file ecm_sensor_component.cpp
 * @brief ECM 组件实现（ESR 假设驱动 + RF-WORLD 发布）。
 */

#include "ecm_sensor_component.h"

#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "core/world.h"
#include "esr_sensor_component.h"
#include "flight_component.h"
#include "logger/logger.h"
#include "rf_world_broker.h"
#include "scene_types.h"
#include "sensor_utils.h"

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

  const auto& scene = static_cast<const DemoSceneState&>(world.scene_state());
  auto& mutable_scene = static_cast<DemoSceneState&>(world.scene_state());

  ecm::session::EcmCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.cycle_start_time_s = scene.t_sec;
  input.dt_sec = dt_sec;
  input.input_mode = ecm::EcmInputMode::kSensorDriven;
  input.platform_entity_id = kDemoPlatformEntityId;
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
  CA_LOG_VIEW("ecm", "周期={} 状态={} 发射数={} ESR批次={}",
              result.input_cycle_index, EcmCycleStatusName(result.status),
              result.emission_frame.emissions.size(),
              static_cast<unsigned long long>(result.source_esr_batch_id));

  if (result.status == ecm::session::EcmCycleStatus::kExecuted) {
    ++executed_cycle_count_;
    if (!result.decisions.empty()) {
      CA_LOG_EVENT(world, "ecm_jamming",
                   "技术={} 决策数={} 功率={:.0f}W",
                   EcmTechniqueName(result.decisions.front().technique),
                   result.decisions.size(), result.decisions.front().allocated_power_w);
    }
    PublishEquipmentEmissions(&mutable_scene, result.emission_frame);
  }
}

}  // namespace component_attachment
