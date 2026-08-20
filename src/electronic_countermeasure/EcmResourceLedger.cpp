/**
 * @file EcmResourceLedger.cpp
 * @brief EcmResourceLedger 实现：发射 ID、热预算、RNG 流和威胁可行性。
 */

#include "electronic_countermeasure/EcmResourceLedger.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>

namespace electronic_countermeasure {
namespace session {

EcmResourceLedger::EcmResourceLedger(std::uint32_t random_seed)
    : scheduling_rng_(DeriveStreamSeed(random_seed, kSchedulingDomain)),
      tie_break_rng_(DeriveStreamSeed(random_seed, kTieBreakDomain)) {}

std::uint64_t EcmResourceLedger::next_emission_id() const { return next_emission_id_; }

std::uint64_t EcmResourceLedger::ReserveEmissionId() {
  const std::uint64_t id = next_emission_id_;
  ++next_emission_id_;
  return id;
}

void EcmResourceLedger::SetNextEmissionId(std::uint64_t id) { next_emission_id_ = id; }

double EcmResourceLedger::thermal_energy_j() const { return thermal_energy_j_; }

void EcmResourceLedger::ApplyCooling(double cooling_power_w, double dt_sec) {
  thermal_energy_j_ = std::max(0.0, thermal_energy_j_ - cooling_power_w * dt_sec);
}

void EcmResourceLedger::AddThermalEnergy(double energy_j) { thermal_energy_j_ += energy_j; }

void EcmResourceLedger::SetThermalEnergy(double energy_j) { thermal_energy_j_ = energy_j; }

double EcmResourceLedger::ComputeThermalPowerLimit(double thermal_capacity_j,
                                                    double dt_sec) const {
  return std::max(0.0, thermal_capacity_j - thermal_energy_j_) / dt_sec;
}

std::mt19937& EcmResourceLedger::scheduling_rng() { return scheduling_rng_; }
const std::mt19937& EcmResourceLedger::scheduling_rng() const { return scheduling_rng_; }

std::mt19937& EcmResourceLedger::tie_break_rng() { return tie_break_rng_; }
const std::mt19937& EcmResourceLedger::tie_break_rng() const { return tie_break_rng_; }

bool EcmResourceLedger::IsFeasibleThreat(const SchedulingThreat& threat,
                                          const config::EcmSessionConfig& config) {
  if (config.default_technique == EcmTechnique::kDeception) {
    double doppler_margin_hz = 0.0;
    switch (config.default_deception_mode) {
      case EcmDeceptionMode::kRgpo:
        doppler_margin_hz = 0.0;
        break;
      case EcmDeceptionMode::kVgpo:
      case EcmDeceptionMode::kRgpoVgpo:
        doppler_margin_hz = config.deception_vgpo_max_doppler_hz;
        break;
      case EcmDeceptionMode::kFalseTarget:
        doppler_margin_hz = std::abs(config.deception_false_target_doppler_hz);
        break;
      default:
        break;
    }
    const double occupied_half_bw_hz = 0.5 * std::max(1000000.0, threat.bandwidth_hz);
    return (threat.center_frequency_hz - occupied_half_bw_hz - doppler_margin_hz) >=
               config.minimum_frequency_hz &&
           (threat.center_frequency_hz + occupied_half_bw_hz + doppler_margin_hz) <=
               config.maximum_frequency_hz;
  }
  double occupied_bandwidth_hz = config.spot_bandwidth_hz;
  if (config.default_technique == EcmTechnique::kBarrage) {
    occupied_bandwidth_hz = std::max(config.barrage_bandwidth_hz, threat.bandwidth_hz);
  } else if (config.default_technique == EcmTechnique::kSweep) {
    occupied_bandwidth_hz = config.sweep_bandwidth_hz;
  }
  return threat.center_frequency_hz - 0.5 * occupied_bandwidth_hz >= config.minimum_frequency_hz &&
         threat.center_frequency_hz + 0.5 * occupied_bandwidth_hz <= config.maximum_frequency_hz;
}

void EcmResourceLedger::AssignTieBreakKeys(std::vector<SchedulingThreat>* threats) {
  if (threats == nullptr || threats->empty()) {
    return;
  }
  std::set<std::uint64_t> unique_ids;
  for (const SchedulingThreat& threat : *threats) {
    unique_ids.insert(ThreatStableId(threat));
  }
  std::map<std::uint64_t, std::uint32_t> keys;
  std::uniform_int_distribution<std::uint32_t> distribution;
  for (std::uint64_t id : unique_ids) {
    const std::uint32_t draw = distribution(tie_break_rng_);
    keys[id] = DeriveStreamSeed(static_cast<std::uint32_t>(id), draw);
  }
  for (SchedulingThreat& threat : *threats) {
    threat.tie_break_key = keys[ThreatStableId(threat)];
  }
}

std::string EcmResourceLedger::SerializeSchedulingRng() const {
  std::ostringstream stream;
  stream << scheduling_rng_;
  return stream.str();
}

std::string EcmResourceLedger::SerializeTieBreakRng() const {
  std::ostringstream stream;
  stream << tie_break_rng_;
  return stream.str();
}

bool EcmResourceLedger::DeserializeSchedulingRng(const std::string& state) {
  std::istringstream stream(state);
  std::mt19937 candidate;
  if (!(stream >> candidate)) {
    return false;
  }
  scheduling_rng_ = candidate;
  return true;
}

bool EcmResourceLedger::DeserializeTieBreakRng(const std::string& state) {
  std::istringstream stream(state);
  std::mt19937 candidate;
  if (!(stream >> candidate)) {
    return false;
  }
  tie_break_rng_ = candidate;
  return true;
}

}  // namespace session
}  // namespace electronic_countermeasure
