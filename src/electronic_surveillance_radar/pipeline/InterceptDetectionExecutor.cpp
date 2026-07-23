#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <utility>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "common/numerics/NumericGuard.h"
#include "common/timing/TimingRegimeModel.h"
#include "common/validation/ValidationUtils.h"
#include "electronic_surveillance_radar/pipeline/EsrRfV2FrontEnd.h"
#include "electronic_surveillance_radar/pipeline/InterceptComponentFactory.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

using oneq::common::numerics::kNumericFloor;

constexpr std::uint64_t kDetectionRandomDomain = 0x4553524454454354ULL;
constexpr std::uint64_t kAngleRandomDomain = 0x455352414e474c45ULL;

std::uint64_t Mix64(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

std::mt19937 MakeEmissionRandomStream(unsigned int session_seed, std::uint32_t cycle_index,
                                      const oneq::electromagnetics::RfEmissionIdentity& identity,
                                      std::uint64_t domain) {
  std::uint64_t seed = Mix64(static_cast<std::uint64_t>(session_seed));
  seed ^= Mix64(static_cast<std::uint64_t>(cycle_index));
  seed ^= Mix64(identity.platform_id);
  seed ^= Mix64(identity.equipment_id);
  seed ^= Mix64(identity.emission_id);
  seed ^= Mix64(domain);
  return std::mt19937(static_cast<std::mt19937::result_type>(seed ^ (seed >> 32U)));
}

float ToDb(double ratio) {
  return static_cast<float>(10.0 * std::log10(std::max(ratio, kNumericFloor)));
}

oneq::common::timing::IntegrationMode ToTimingIntegrationMode(
    extension::InterceptIntegrationMode mode) {
  return mode == extension::InterceptIntegrationMode::kCoherent
             ? oneq::common::timing::IntegrationMode::kCoherent
             : oneq::common::timing::IntegrationMode::kNonCoherent;
}

oneq::common::timing::StatisticalDetectionParams ToTimingDetectionParams(
    const extension::InterceptStatisticalDetectionConfig& config) {
  oneq::common::timing::StatisticalDetectionParams params;
  params.pfa = config.pfa;
  params.min_snr_db = config.min_snr_db;
  params.pulse_count = config.pulse_count;
  params.integration_mode = ToTimingIntegrationMode(config.integration_mode);
  params.threshold_scale = config.threshold_scale;
  params.enable_statistical_detection = config.enable_statistical_detection;
  return params;
}

std::pair<double, double> BuildReceiverWindow(
    std::uint64_t completed_receive_cycles,
    const extension::InterceptRuntimeConfig& runtime_config) {
  const std::vector<config::EsrTuningWindow>& plan = runtime_config.receiver_hardware.tuning_plan;
  std::uint64_t total_dwell = 0U;
  for (const config::EsrTuningWindow& window : plan) {
    total_dwell += window.dwell_cycles;
  }
  if (total_dwell > 0U) {
    std::uint64_t phase = completed_receive_cycles % total_dwell;
    for (const config::EsrTuningWindow& window : plan) {
      if (phase < window.dwell_cycles) {
        return std::make_pair(window.center_frequency_hz - 0.5 * window.bandwidth_hz,
                              window.center_frequency_hz + 0.5 * window.bandwidth_hz);
      }
      phase -= window.dwell_cycles;
    }
  }
  if (runtime_config.use_fixed_receiver_window &&
      oneq::common::validation::IsFinite(runtime_config.receiver_lower_hz) &&
      oneq::common::validation::IsFinite(runtime_config.receiver_upper_hz) &&
      runtime_config.receiver_upper_hz > runtime_config.receiver_lower_hz) {
    return std::make_pair(runtime_config.receiver_lower_hz, runtime_config.receiver_upper_hz);
  }
  return std::make_pair(runtime_config.receiver_hardware.receiver_band_lower_hz,
                        runtime_config.receiver_hardware.receiver_band_upper_hz);
}

std::size_t ResolveActiveBeamIndex(double* scan_phase_cycles, float dt_sec,
                                   std::size_t scan_pattern_size,
                                   const extension::InterceptRuntimeConfig& runtime_config) {
  if (scan_pattern_size == 0U || scan_phase_cycles == nullptr) {
    return 0U;
  }
  const double safe_dt = std::isfinite(dt_sec) && dt_sec > 0.0f ? dt_sec : 0.0;
  const double scan_rate = std::isfinite(runtime_config.scan_rate_hz) &&
                                   runtime_config.scan_rate_hz > 0.0f
                               ? runtime_config.scan_rate_hz
                               : 1.0;
  const double phase = std::min(std::nextafter(1.0, 0.0), std::max(0.0, *scan_phase_cycles));
  const std::size_t index = static_cast<std::size_t>(
      std::floor(phase * static_cast<double>(scan_pattern_size)));
  *scan_phase_cycles = std::fmod(phase + scan_rate * safe_dt, 1.0);
  return index;
}

session::EsrObservationQuality ClassifyObservationQuality(float snr_db) {
  return snr_db >= 18.0f ? session::EsrObservationQuality::kHigh
         : snr_db >= 10.0f ? session::EsrObservationQuality::kMedium
                           : session::EsrObservationQuality::kLow;
}

double ResolveCenterFrequencyHz(const oneq::electromagnetics::RfIncidentLinkResult& link) {
  const oneq::electromagnetics::RfWaveformSchedule& waveform = link.emission_waveform;
  if (waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep) {
    return 0.5 * (waveform.sweep_start_frequency_hz + waveform.sweep_stop_frequency_hz) +
           link.doppler_shift_hz;
  }
  return waveform.center_frequency_hz + link.doppler_shift_hz;
}

double ResolveChannelPowerW(const oneq::electromagnetics::RfIncidentLinkResult& source,
                            double channel_center_hz, double channel_bandwidth_hz) {
  if (!std::isfinite(channel_center_hz) || !std::isfinite(channel_bandwidth_hz) ||
      channel_bandwidth_hz <= 0.0 || source.received_power_spectral_density_w_per_hz <= 0.0) {
    return 0.0;
  }
  const double source_center = ResolveCenterFrequencyHz(source);
  const double source_bandwidth = source.emission_waveform.occupied_bandwidth_hz;
  const double lower = std::max(channel_center_hz - 0.5 * channel_bandwidth_hz,
                                source_center - 0.5 * source_bandwidth);
  const double upper = std::min(channel_center_hz + 0.5 * channel_bandwidth_hz,
                                source_center + 0.5 * source_bandwidth);
  return upper > lower ? source.received_power_spectral_density_w_per_hz * (upper - lower) *
                             source.time_overlap_fraction
                       : 0.0;
}

bool TryResolveLookAngles(const oneq::electromagnetics::RfSceneReceiverState& receiver,
                          const oneq::electromagnetics::RfIncidentLinkResult& link,
                          const oneq::electromagnetics::RfSceneEmission& emission,
                          const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
                          double* azimuth_deg,
                          double* elevation_deg) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr || !std::isfinite(link.path_length_m) ||
      link.path_length_m <= 0.0 || link.is_co_site) {
    return false;
  }
  oneq::coordinate::LlaPositionDegM receiver_lla;
  oneq::coordinate::EnuPositionM enu;
  if (!oneq::coordinate::TryEcefToLla(receiver.position_ecef_m, &receiver_lla) ||
      !oneq::coordinate::TryEcefToEnu(emission.position_ecef_m, receiver_lla, &enu)) {
    return false;
  }
  const oneq::coordinate::Vector3d local = oneq::coordinate::RotateEnuToLocal(
      enu.east_m, enu.north_m, enu.up_m, platform_attitude_deg);
  const double horizontal = std::hypot(local.x, local.y);
  if (!std::isfinite(horizontal) || horizontal <= 0.0) {
    return false;
  }
  *azimuth_deg = std::atan2(local.y, local.x) * 180.0 / 3.14159265358979323846;
  *elevation_deg = std::atan2(local.z, horizontal) * 180.0 / 3.14159265358979323846;
  return std::isfinite(*azimuth_deg) && std::isfinite(*elevation_deg);
}

struct ArrivalBearing {
  bool defined{false};
  double azimuth_deg{0.0};
  double elevation_deg{0.0};
};

bool IsAngularResolutionCellShared(const ArrivalBearing& left, const ArrivalBearing& right,
                                   double beamwidth_deg) {
  if (!left.defined || !right.defined || !std::isfinite(beamwidth_deg) || beamwidth_deg <= 0.0) {
    return true;
  }
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double left_azimuth = left.azimuth_deg * kDegToRad;
  const double left_elevation = left.elevation_deg * kDegToRad;
  const double right_azimuth = right.azimuth_deg * kDegToRad;
  const double right_elevation = right.elevation_deg * kDegToRad;
  const double cosine =
      std::sin(left_elevation) * std::sin(right_elevation) +
      std::cos(left_elevation) * std::cos(right_elevation) * std::cos(left_azimuth - right_azimuth);
  const double separation_deg = std::acos(std::max(-1.0, std::min(1.0, cosine))) / kDegToRad;
  return std::isfinite(separation_deg) && separation_deg < beamwidth_deg;
}

}  // namespace

InterceptDetectionOutput InterceptDetectionExecutor::Execute(
    const MutableEsrContext& ctx, std::uint64_t& next_observation_id, double* scan_phase_cycles,
    std::uint64_t completed_receive_cycles) {
  InterceptDetectionOutput output;
  output.scan_pattern = intercept::ScanPatternGenerator::Generate(
      InterceptComponentFactory::BuildScanPatternConfig(ctx.GetPipelineConfig()));
  if (output.scan_pattern.empty()) {
    output.scan_pattern.push_back(intercept::BeamPointingDeg());
  }
  const intercept::BeamPointingDeg active_beam =
      output
          .scan_pattern[ResolveActiveBeamIndex(scan_phase_cycles, ctx.GetCycleDeltaTimeSec(),
                                               output.scan_pattern.size(), ctx.GetRuntimeConfig())];
  const std::pair<double, double> receiver_window =
      BuildReceiverWindow(completed_receive_cycles, ctx.GetRuntimeConfig());
  output.receiver_center_frequency_hz = 0.5 * (receiver_window.first + receiver_window.second);
  output.receiver_bandwidth_hz = receiver_window.second - receiver_window.first;
  if (!ProcessRfV2Frame(
          ctx, active_beam, receiver_window,
          InterceptComponentFactory::BuildAngleErrorModelConfig(ctx.GetPipelineConfig()),
          ToTimingDetectionParams(ctx.GetPipelineConfig().statistical_detection),
          next_observation_id, &output)) {
    output.rf_v2_rejected = true;
  }
  return output;
}

bool InterceptDetectionExecutor::ProcessRfV2Frame(
    const MutableEsrContext& ctx, const intercept::BeamPointingDeg& active_beam,
    const std::pair<double, double>& receiver_window,
    const intercept::AngleErrorModelConfig& angle_error_config,
    const oneq::common::timing::StatisticalDetectionParams& base_detection,
    std::uint64_t& next_observation_id, InterceptDetectionOutput* output) const {
  if (output == nullptr) {
    return false;
  }
  EsrRfV2FrontEndResult front_end;
  if (!TryResolveEsrRfV2FrontEnd(
          ctx.GetCycleInput(), ctx.GetRuntimeConfig().receiver_hardware, active_beam.az_deg,
          active_beam.el_deg, 0.5 * (receiver_window.first + receiver_window.second),
          receiver_window.second - receiver_window.first,
          std::max(0.0f, ctx.GetEnvironmentSnapshot().propagation_loss_db), &front_end)) {
    return false;
  }
  output->receiver_center_frequency_hz = front_end.receiver.center_frequency_hz;
  output->receiver_bandwidth_hz = front_end.receiver.bandwidth_hz;
  output->receiver_saturated = front_end.receiver_saturated;
  if (front_end.receiver_saturated) {
    return true;
  }
  std::vector<const oneq::electromagnetics::RfSceneEmission*> emissions;
  for (const oneq::electromagnetics::RfSceneEmission& emission : ctx.GetInterference().emissions) {
    emissions.push_back(&emission);
  }
  std::sort(emissions.begin(), emissions.end(),
            [](const oneq::electromagnetics::RfSceneEmission* a,
               const oneq::electromagnetics::RfSceneEmission* b) {
              return a->identity.platform_id != b->identity.platform_id
                         ? a->identity.platform_id < b->identity.platform_id
                     : a->identity.equipment_id != b->identity.equipment_id
                         ? a->identity.equipment_id < b->identity.equipment_id
                         : a->identity.emission_id < b->identity.emission_id;
            });
  if (emissions.size() != front_end.incident_links.size()) {
    return false;
  }
  constexpr double kBoltzmannJPerK = 1.380649e-23;
  const config::EsrHardwareConfig& hardware = ctx.GetRuntimeConfig().receiver_hardware;
  const double thermal_noise = kBoltzmannJPerK * hardware.receiver_reference_temperature_k *
                               front_end.receiver.bandwidth_hz *
                               std::pow(10.0, hardware.receiver_noise_figure_db / 10.0);
  const double ambient_noise =
      std::max(thermal_noise + static_cast<double>(ctx.GetEnvironmentSnapshot().clutter_noise_w),
               kNumericFloor);
  const double beamwidth = std::max(1.0, std::max(static_cast<double>(hardware.beam_az_width_deg),
                                                  static_cast<double>(hardware.beam_el_width_deg)));
  std::vector<ArrivalBearing> bearings(front_end.incident_links.size());
  for (std::size_t index = 0U; index < front_end.incident_links.size(); ++index) {
    if (front_end.incident_links[index].is_co_site ||
        front_end.incident_links[index].received_power_w <= 0.0) {
      continue;
    }
    ArrivalBearing& bearing = bearings[index];
    if (!TryResolveLookAngles(front_end.receiver, front_end.incident_links[index],
                              *emissions[index], ctx.GetPlatformAttitude(), &bearing.azimuth_deg,
                              &bearing.elevation_deg)) {
      return false;
    }
    bearing.defined = true;
  }
  for (std::size_t signal_index = 0U; signal_index < front_end.incident_links.size();
       ++signal_index) {
    const oneq::electromagnetics::RfIncidentLinkResult& signal =
        front_end.incident_links[signal_index];
    if (signal.received_power_w <= 0.0) {
      continue;
    }
    // 同平台发射可进入前端噪声/饱和账本，却没有可定义的外部 AoA；将其作为
    // ESR 发射源观测发布会伪造方向信息。它仍保留在下面对其他入射信号的干扰求和中。
    if (signal.is_co_site) {
      continue;
    }
    const double center_hz = ResolveCenterFrequencyHz(signal);
    const double bandwidth_hz = signal.emission_waveform.occupied_bandwidth_hz;
    double interference = 0.0;
    for (std::size_t other = 0U; other < front_end.incident_links.size(); ++other) {
      if (other != signal_index &&
          IsAngularResolutionCellShared(bearings[signal_index], bearings[other], beamwidth)) {
        interference +=
            ResolveChannelPowerW(front_end.incident_links[other], center_hz, bandwidth_hz);
      }
    }
    const double snr_db =
        ToDb(signal.received_power_w / std::max(ambient_noise + interference, kNumericFloor));
    oneq::common::timing::StatisticalDetectionParams detection = base_detection;
    detection.pulse_count =
        signal.emission_waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain
            ? std::max(1U,
                       static_cast<std::uint32_t>(std::round(signal.emission_waveform.pulse_count *
                                                             signal.time_overlap_fraction)))
            : 1U;
    const float threshold =
        ctx.GetPipelineConfig().statistical_detection.enable_statistical_detection
            ? std::max(ctx.GetPipelineConfig().detection.minimum_snr_db,
                       oneq::common::timing::ComputeDynamicThresholdSnrDb(
                           ambient_noise + interference, detection))
            : ctx.GetPipelineConfig().detection.minimum_snr_db;
    std::mt19937 detection_rng = MakeEmissionRandomStream(
        ctx.GetPipelineConfig().algorithm.random_seed, ctx.GetCycleInput().cycle_index,
        signal.identity, kDetectionRandomDomain);
    std::uniform_real_distribution<float> uniform_01(0.0f, 1.0f);
    if (snr_db < threshold ||
        (ctx.GetPipelineConfig().statistical_detection.enable_statistical_detection &&
         uniform_01(detection_rng) >= oneq::common::timing::ComputeStatisticalDetectionProbability(
                                          static_cast<float>(snr_db), threshold, detection))) {
      continue;
    }
    const double relative_std = std::max(
        1.0e-6, std::min(0.25, 1.0 / std::sqrt(std::max(std::pow(10.0, snr_db / 10.0), 1.0e-12))));
    RawObservationRecord record;
    std::mt19937 angle_rng = MakeEmissionRandomStream(ctx.GetPipelineConfig().algorithm.random_seed,
                                                      ctx.GetCycleInput().cycle_index,
                                                      signal.identity, kAngleRandomDomain);
    record.observation.observation_id = next_observation_id++;
    record.observation.timestamp_s = signal.arrival_start_time_s;
    record.observation.aoa_az_deg = bearings[signal_index].azimuth_deg +
                                    intercept::AngleErrorModel::SampleErrorDeg(
                                        static_cast<float>(snr_db), static_cast<float>(beamwidth),
                                        &angle_rng, angle_error_config);
    record.observation.aoa_el_deg = bearings[signal_index].elevation_deg +
                                    intercept::AngleErrorModel::SampleErrorDeg(
                                        static_cast<float>(snr_db), static_cast<float>(beamwidth),
                                        &angle_rng, angle_error_config);
    record.observation.rf_hz = center_hz;
    record.observation.bandwidth_hz = bandwidth_hz;
    if (signal.emission_waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
      record.observation.pri_s = signal.emission_waveform.pulse_repetition_interval_s;
      record.observation.pulse_width_s = signal.emission_waveform.pulse_width_s;
    }
    record.observation.rf_std_hz = std::max(1.0, bandwidth_hz * relative_std);
    record.observation.bandwidth_std_hz = std::max(1.0, bandwidth_hz * relative_std);
    record.observation.pri_std_s = std::max(1.0e-12, record.observation.pri_s * relative_std);
    record.observation.pulse_width_std_s =
        std::max(1.0e-12, record.observation.pulse_width_s * relative_std);
    record.observation.amplitude_db = ToDb(signal.received_power_w);
    record.observation.snr_db = snr_db;
    record.observation.quality = ClassifyObservationQuality(static_cast<float>(snr_db));
    output->raw_records.push_back(record);
  }
  return true;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
