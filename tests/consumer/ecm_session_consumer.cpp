/**
 * @file ecm_session_consumer.cpp
 * @brief 验证安装后的 ECM、ESR adapter 与公共 RF 发射事实可被 C++11 消费者使用。
 */

#include <vector>

#include "1q/electronic_countermeasure/EcmEsrAdapter.h"
#include "1q/electronic_countermeasure/EcmSession.h"
#include "1q/threat_assessment/ThreatEvaluationInput.h"
#include "1q/threat_assessment/ThreatEvaluator.h"
#include "1q/threat_assessment/ThreatEvaluatorConfig.h"

namespace ecm = electronic_countermeasure;
namespace esr_session = electronic_surveillance_radar::session;

int main() {
  esr_session::EmitterHypothesis hypothesis;
  hypothesis.hypothesis_id = 17U;
  hypothesis.mode = esr_session::EsrEmitterMode::kGuidance;
  hypothesis.estimated_center_frequency_hz = 9.3e9;
  hypothesis.estimated_bandwidth_hz = 5.0e6;
  hypothesis.estimated_pri_s = 1.0e-3;
  hypothesis.estimated_pulse_width_s = 1.0e-6;
  hypothesis.center_frequency_std_hz = 1.0e5;
  hypothesis.bandwidth_std_hz = 1.0e5;
  hypothesis.bearing_std_deg = 2.0f;
  hypothesis.confidence = 0.9f;

  esr_session::EmitterHypothesisList hypotheses;
  hypotheses.push_back(hypothesis);

  // 决策层供给链（TARGET-OQ-2 处置）：威胁分由 threat_assessment 产出后值级注入 ECM。
  threat_assessment::ThreatEvaluatorConfig threat_config;
  threat_config.weight_range = 0.0f;
  threat_config.weight_speed = 0.0f;
  threat_config.weight_acceleration = 0.0f;
  threat_config.weight_rcs = 0.0f;
  threat_config.weight_target_probability = 0.0f;
  threat_config.weight_fusion_confidence = 0.0f;
  threat_config.weight_emitter_threat_evidence = 1.0f;
  const threat_assessment::ThreatEvaluator evaluator(threat_config);
  threat_assessment::ThreatEvaluationInput threat_input;
  threat_input.key = hypothesis.hypothesis_id;
  threat_input.emitter_threat_evidence = 1.0f * hypothesis.confidence;
  const std::vector<threat_assessment::ThreatResult> threat_results =
      evaluator.Evaluate({threat_input});
  std::vector<float> threat_scores;
  for (const threat_assessment::ThreatResult& result : threat_results) {
    threat_scores.push_back(result.threat_score);
  }

  ecm::session::EcmSensorObservationFrame frame;
  if (!ecm::session::TryBuildEcmSensorObservationFrame(hypotheses, 400U, threat_scores,
                                                       &frame)) {
    return 1;
  }

  ecm::config::EcmSessionConfig config;
  config.channel_count = 1U;
  config.maximum_total_transmit_power_w = 500.0;
  config.maximum_channel_transmit_power_w = 500.0;
  ecm::session::EcmSession session = ecm::session::EcmSession::Create(config);

  ecm::session::EcmCycleInput input;
  input.cycle_index = 5U;
  input.cycle_start_time_s = 4.0;
  input.input_mode = ecm::EcmInputMode::kSensorDriven;
  input.platform_entity_id = 99U;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.has_sensor_observation_frame = true;
  input.sensor_observation_frame = frame;
  const ecm::session::EcmCycleResult result = session.StepWithResult(input);
  if (result.status != ecm::session::EcmCycleStatus::kExecuted ||
      result.source_esr_batch_id != 400U ||
      result.emission_frame.emissions.empty()) {
    return 2;
  }
  return 0;
}
