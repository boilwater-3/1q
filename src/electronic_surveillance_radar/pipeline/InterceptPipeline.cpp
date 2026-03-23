#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "electronic_surveillance_radar/intercept/AngleErrorModel.h"
#include "electronic_surveillance_radar/intercept/BoundarySearchSolver.h"
#include "electronic_surveillance_radar/intercept/InterceptGate.h"
#include "electronic_surveillance_radar/intercept/JammingAggregator.h"
#include "electronic_surveillance_radar/intercept/ScanPatternGenerator.h"
#include "electronic_surveillance_radar/pipeline/InterceptComponentFactory.h"
#include "electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h"

namespace electronic_surveillance_radar {
namespace pipeline {

namespace {

/**
 * @brief 电磁传播计算中使用的圆周率常量。
 * @note 代码行为依据：接收功率计算使用自由空间路径损耗模型中的 `4πR/λ` 项。
 */
constexpr double kPi = 3.14159265358979323846;
/**
 * @brief 以米每秒表示的光速常量。
 * @note 代码行为依据：波长通过 `λ = c/f` 计算，用于路径损耗估计。
 */
constexpr double kLightSpeedMps = 299792458.0;
/**
 * @brief 数值稳定保护下限。
 * @note 代码行为依据：用于避免除零、`log10(0)` 与非正噪声功率。
 */
constexpr double kNumericFloor = 1.0e-18;

/**
 * @brief 将输入裁剪到 [0, 1]。
 * @param[in] value 输入值。
 * @return 裁剪后结果。
 */
float Clamp01(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

/**
 * @brief 把线性功率比转换为 dB。
 * @param[in] ratio 线性功率比。
 * @return dB 表示值。
 */
float ToDb(double ratio) {
  return static_cast<float>(10.0 * std::log10(std::max(ratio, kNumericFloor)));
}

/**
 * @brief 计算平台与辐射源之间的欧氏距离。
 * @param[in] platform_pose 平台状态。
 * @param[in] emitter_state 辐射源状态。
 * @return 两者距离（单位：m）。
 */
float ComputeRangeM(const common::EsrPoseState& platform_pose,
                    const common::EmitterTruthState& emitter_state) {
  const float dx = emitter_state.pose.position_m.x - platform_pose.position_m.x;
  const float dy = emitter_state.pose.position_m.y - platform_pose.position_m.y;
  const float dz = emitter_state.pose.position_m.z - platform_pose.position_m.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 计算辐射源相对平台的方位角。
 * @param[in] platform_pose 平台状态。
 * @param[in] emitter_state 辐射源状态。
 * @return 方位角（单位：deg）。
 */
float ComputeAzimuthDeg(const common::EsrPoseState& platform_pose,
                        const common::EmitterTruthState& emitter_state) {
  const float dx = emitter_state.pose.position_m.x - platform_pose.position_m.x;
  const float dy = emitter_state.pose.position_m.y - platform_pose.position_m.y;
  return static_cast<float>(std::atan2(dy, dx) * 180.0 / kPi);
}

/**
 * @brief 计算辐射源相对平台的俯仰角。
 * @param[in] platform_pose 平台状态。
 * @param[in] emitter_state 辐射源状态。
 * @return 俯仰角（单位：deg）。
 */
float ComputeElevationDeg(const common::EsrPoseState& platform_pose,
                          const common::EmitterTruthState& emitter_state) {
  const float dx = emitter_state.pose.position_m.x - platform_pose.position_m.x;
  const float dy = emitter_state.pose.position_m.y - platform_pose.position_m.y;
  const float dz = emitter_state.pose.position_m.z - platform_pose.position_m.z;
  const float horizontal = std::sqrt(dx * dx + dy * dy);
  return static_cast<float>(std::atan2(dz, horizontal) * 180.0 / kPi);
}

/**
 * @brief 计算自由空间模型下的接收功率。
 * @param[in] tx_power_w 发射功率（单位：W）。
 * @param[in] carrier_hz 载频（单位：Hz）。
 * @param[in] range_m 距离（单位：m）。
 * @param[in] propagation_loss_db 额外传播损耗（单位：dB）。
 * @return 接收功率（单位：W）。
 */
double ComputeReceivedPowerW(double tx_power_w, double carrier_hz, float range_m,
                             float propagation_loss_db) {
  if (tx_power_w <= 0.0 || carrier_hz <= 0.0) {
    return 0.0;
  }
  const double safe_range = std::max(static_cast<double>(range_m), 1.0);
  const double wavelength = kLightSpeedMps / carrier_hz;
  const double fspl_linear =
      std::pow((4.0 * kPi * safe_range) / wavelength, 2.0);
  const double propagation_loss_linear =
      std::pow(10.0, std::max(0.0f, propagation_loss_db) / 10.0);
  return tx_power_w / (fspl_linear * propagation_loss_linear);
}

/**
 * @brief 构造按周期轮换的接收机工作频段窗口。
 * @param[in] cycle_index 当前周期号。
 * @return 接收机频段 `[lower, upper]`，单位 Hz。
 */
std::pair<double, double> BuildReceiverWindow(std::uint32_t cycle_index) {
  struct BandWindow {
    double lower_hz;
    double upper_hz;
  };
  static const BandWindow kBandWindows[] = {
      {0.23e9, 1.0e9},   {1.0e9, 2.0e9},   {2.0e9, 4.0e9},   {4.0e9, 8.0e9},
      {8.0e9, 12.0e9},   {12.0e9, 18.0e9}, {18.0e9, 27.0e9}, {27.0e9, 40.0e9},
      {40.0e9, 60.0e9},  {60.0e9, 100.0e9}};
  const std::size_t window_index =
      static_cast<std::size_t>(cycle_index) %
      (sizeof(kBandWindows) / sizeof(kBandWindows[0]));
  return std::make_pair(kBandWindows[window_index].lower_hz,
                        kBandWindows[window_index].upper_hz);
}

/**
 * @brief 按观测条件映射观测质量等级。
 * @param[in] snr_db 观测信噪比（单位：dB）。
 * @param[in] is_jammed 是否受干扰显著影响。
 * @return 观测质量等级。
 */
common::ObservationQuality ClassifyObservationQuality(float snr_db,
                                                      bool is_jammed) {
  if (!is_jammed && snr_db >= 18.0f) {
    return common::ObservationQuality::kHigh;
  }
  if (snr_db >= 10.0f) {
    return common::ObservationQuality::kMedium;
  }
  return common::ObservationQuality::kLow;
}

/**
 * @brief 构造观测级置信度。
 * @param[in] snr_db 观测信噪比（单位：dB）。
 * @param[in] is_jammed 是否受扰。
 * @return 置信度，范围 [0, 1]。
 */
float ComputeObservationConfidence(float snr_db, bool is_jammed) {
  const float snr_score = Clamp01((snr_db + 5.0f) / 30.0f);
  const float jam_penalty = is_jammed ? 0.75f : 1.0f;
  return Clamp01(snr_score * jam_penalty);
}

/**
 * @brief 构造簇级摘要。
 * @param[in] cluster_indices 簇内观测索引。
 * @param[in] records 预处理后的观测记录。
 * @param[in] features 观测特征向量。
 * @return 簇摘要。
 */
internal::ClusterSummary BuildClusterSummary(
    const std::vector<std::size_t>& cluster_indices,
    const std::vector<internal::RawObservationRecord>& records,
    const std::vector<internal::ObservationFeatureVector>& features,
    const InterceptDeceptionModelConfig& deception_config) {
  internal::ClusterSummary summary;
  if (cluster_indices.empty()) {
    return summary;
  }

  summary.support_count = cluster_indices.size();
  std::size_t representative = cluster_indices[0];
  float representative_snr = records[representative].observation.snr_db;
  float confidence_acc = 0.0f;
  std::size_t deception_affected_count = 0U;
  std::size_t false_alarm_count = 0U;

  for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
    const std::size_t index = cluster_indices[i];
    for (std::size_t dim = 0; dim < internal::kObservationFeatureDimension; ++dim) {
      summary.centroid_feature.values[dim] += features[index].values[dim];
    }
    summary.mean_snr_db += records[index].observation.snr_db;
    summary.mean_az_deg += records[index].observation.aoa_az_deg;
    summary.mean_el_deg += records[index].observation.aoa_el_deg;
    summary.mean_rf_hz += records[index].observation.rf_hz;
    summary.mean_pulse_width_s += records[index].observation.pulse_width_s;
    confidence_acc += ComputeObservationConfidence(
        records[index].observation.snr_db, records[index].observation.is_jammed);
    if (records[index].deception_affected) {
      ++deception_affected_count;
    }
    if (records[index].synthetic_false_alarm) {
      ++false_alarm_count;
    }
    summary.any_jammed =
        summary.any_jammed || records[index].observation.is_jammed;
    if (records[index].observation.snr_db > representative_snr) {
      representative = index;
      representative_snr = records[index].observation.snr_db;
    }
  }

  const float inv_count = 1.0f / static_cast<float>(cluster_indices.size());
  for (std::size_t dim = 0; dim < internal::kObservationFeatureDimension; ++dim) {
    summary.centroid_feature.values[dim] *= inv_count;
  }
  summary.mean_snr_db *= inv_count;
  summary.mean_az_deg *= inv_count;
  summary.mean_el_deg *= inv_count;
  summary.mean_rf_hz *= static_cast<double>(inv_count);
  summary.mean_pulse_width_s *= static_cast<double>(inv_count);
  summary.deception_support_ratio =
      static_cast<float>(deception_affected_count) * inv_count;
  summary.false_alarm_ratio = static_cast<float>(false_alarm_count) * inv_count;
  const float deception_penalty = 1.0f - Clamp01(
                                             deception_config
                                                 .cluster_confidence_penalty_scale *
                                             summary.deception_support_ratio);
  summary.confidence_score = Clamp01(confidence_acc * inv_count * deception_penalty);
  summary.representative_index = representative;
  return summary;
}

/**
 * @brief 对真实观测施加欺骗式错分选扰动。
 * @param[in] deception_strength 欺骗强度。
 * @param[in] config 欺骗建模配置。
 * @param[in,out] rng 随机引擎。
 * @param[in,out] record 待扰动观测。
 */
void ApplyDeceptionConfusion(float deception_strength,
                             const InterceptDeceptionModelConfig& config,
                             std::mt19937* rng,
                             internal::RawObservationRecord* record) {
  if (rng == nullptr || record == nullptr) {
    return;
  }
  const float strength = Clamp01(deception_strength);
  const float aoa_std_deg = std::max(0.1f, config.aoa_confusion_std_deg);
  const float rf_ratio = std::max(0.0f, config.rf_confusion_ratio);
  const float pw_ratio = std::max(0.0f, config.pw_confusion_ratio);
  std::normal_distribution<float> az_noise_dist(0.0f, aoa_std_deg * strength);
  std::normal_distribution<float> el_noise_dist(0.0f, aoa_std_deg * 0.6f * strength);
  std::uniform_real_distribution<float> rf_ratio_dist(-rf_ratio, rf_ratio);
  std::uniform_real_distribution<float> pw_ratio_dist(-pw_ratio, pw_ratio);

  record->observation.aoa_az_deg += az_noise_dist(*rng);
  record->observation.aoa_el_deg += el_noise_dist(*rng);
  record->observation.rf_hz +=
      static_cast<double>(record->observation.rf_hz) *
      static_cast<double>(rf_ratio_dist(*rng) * strength);
  record->observation.pulse_width_s +=
      static_cast<double>(record->observation.pulse_width_s) *
      static_cast<double>(pw_ratio_dist(*rng) * strength);
  record->observation.pulse_width_s =
      std::max(record->observation.pulse_width_s, 1.0e-9);
  record->deception_affected = true;
}

/**
 * @brief 生成欺骗式伪观测。
 * @param[in] template_record 模板观测。
 * @param[in] deception_strength 欺骗强度。
 * @param[in] config 欺骗建模配置。
 * @param[in,out] rng 随机引擎。
 * @param[in,out] next_observation_id 观测 ID 分配器。
 * @return 伪观测记录。
 */
internal::RawObservationRecord BuildDeceptionRecord(
    const internal::RawObservationRecord& template_record,
    float deception_strength, const InterceptDeceptionModelConfig& config,
    std::mt19937* rng,
    std::uint64_t* next_observation_id) {
  internal::RawObservationRecord record = template_record;
  if (next_observation_id != nullptr) {
    record.observation.observation_id = (*next_observation_id)++;
  }
  const float strength = Clamp01(deception_strength);
  std::normal_distribution<float> angle_dist(
      0.0f, std::max(0.5f, config.aoa_confusion_std_deg) * (0.5f + strength));
  std::uniform_real_distribution<float> rf_shift_dist(
      -(0.6f + strength), 0.6f + strength);
  std::uniform_real_distribution<float> pw_shift_dist(
      -std::max(0.0f, config.pw_confusion_ratio),
      std::max(0.0f, config.pw_confusion_ratio));
  std::uniform_real_distribution<float> snr_loss_dist(4.0f, 10.0f);
  if (rng != nullptr) {
    record.observation.aoa_az_deg += angle_dist(*rng);
    record.observation.aoa_el_deg += angle_dist(*rng) * 0.6f;
    record.observation.rf_hz += static_cast<double>(template_record.observation.rf_hz) *
                                static_cast<double>(0.01f * rf_shift_dist(*rng));
    record.observation.pulse_width_s +=
        static_cast<double>(template_record.observation.pulse_width_s) *
        static_cast<double>(pw_shift_dist(*rng));
    record.observation.pulse_width_s =
        std::max(record.observation.pulse_width_s, 1.0e-9);
    record.observation.snr_db -= snr_loss_dist(*rng);
  }
  record.observation.quality = common::ObservationQuality::kLow;
  record.observation.is_jammed = false;
  record.truth_emitter_id = "";
  record.matched_truth = false;
  record.deception_affected = true;
  record.synthetic_false_alarm = true;
  return record;
}

}  // namespace

InterceptPipeline::InterceptPipeline(InterceptPipelineConfig config)
    : config_(config),
      associator_(config.association),
      rng_(config.algorithm.random_seed == 0U ? 1U
                                              : config.algorithm.random_seed) {
  feature_scales_.rf_scale_hz = config_.cluster.rf_scale_hz;
  feature_scales_.pw_scale_sec = config_.cluster.pw_scale_sec;
  feature_scales_.az_scale_deg = config_.cluster.az_scale_deg;
  feature_scales_.el_scale_deg = config_.cluster.el_scale_deg;
  feature_scales_.snr_scale_db = config_.cluster.snr_scale_db;
}

InterceptCycleResult InterceptPipeline::RunCycle(
    const core::context::EsrCycleInput& input_state,
    const environment::IEsrEnvironmentService& environment_service) {
  InterceptCycleResult result;
  const environment::EsrEnvironmentSnapshot environment_snapshot =
      environment_service.SampleEnvironment();

  const intercept::ScanPatternConfig scan_pattern_config =
      internal::InterceptComponentFactory::BuildScanPatternConfig(config_);
  std::vector<intercept::BeamPointingDeg> scan_pattern =
      intercept::ScanPatternGenerator::Generate(scan_pattern_config);
  if (scan_pattern.empty()) {
    scan_pattern.push_back(intercept::BeamPointingDeg());
  }
  const intercept::BeamPointingDeg active_beam =
      scan_pattern[static_cast<std::size_t>(input_state.cycle_index) %
                   scan_pattern.size()];

  const intercept::AngleErrorModelConfig angle_error_config =
      internal::InterceptComponentFactory::BuildAngleErrorModelConfig(config_);
  const std::pair<double, double> receiver_window =
      BuildReceiverWindow(input_state.cycle_index);

  std::vector<internal::RawObservationRecord> raw_records;
  raw_records.reserve(
      input_state.scene_emitters.size() *
      (1U + static_cast<std::size_t>(
                config_.deception_model.max_false_observations_per_emitter)));
  std::uniform_real_distribution<float> uniform_01(0.0f, 1.0f);

  for (std::size_t i = 0; i < input_state.scene_emitters.size(); ++i) {
    const common::EmitterTruthState& emitter = input_state.scene_emitters[i];
    if (!emitter.is_emitting || emitter.carrier_hz <= 0.0 ||
        emitter.bandwidth_hz <= 0.0 || emitter.tx_power_w <= 0.0) {
      continue;
    }

    const float range_m = ComputeRangeM(input_state.platform_pose, emitter);
    const float target_az_deg = ComputeAzimuthDeg(input_state.platform_pose, emitter);
    const float target_el_deg =
        ComputeElevationDeg(input_state.platform_pose, emitter);

    const intercept::JammingAggregateResult jamming_result =
        intercept::JammingAggregator::Aggregate(
            environment_snapshot.jammer_sources, emitter.carrier_hz,
            emitter.bandwidth_hz);
    const float suppression_noise_scale =
        std::max(0.0f, config_.suppression_model.suppression_noise_scale);
    const double effective_suppression_power_w =
        static_cast<double>(jamming_result.suppression_power_w) *
        static_cast<double>(suppression_noise_scale);
    const double noise_power_w =
        std::max(static_cast<double>(config_.detection.receiver_noise_floor_w) +
                     static_cast<double>(environment_snapshot.clutter_noise_w) +
                     effective_suppression_power_w,
                 kNumericFloor);

    const intercept::BoundarySearchResult boundary_result =
        intercept::BoundarySearchSolver::Solve(
            1.0f, config_.detection.max_detect_range_m,
            config_.detection.boundary_resolution_m,
            config_.detection.boundary_max_iterations,
            [&](float candidate_range_m) {
              const double candidate_received_power_w = ComputeReceivedPowerW(
                  emitter.tx_power_w, emitter.carrier_hz, candidate_range_m,
                  environment_snapshot.propagation_loss_db);
              const float candidate_snr_db =
                  ToDb(candidate_received_power_w / noise_power_w);
              return candidate_snr_db >= config_.detection.min_detect_snr_db;
            });
    const double received_power_w = ComputeReceivedPowerW(
        emitter.tx_power_w, emitter.carrier_hz, range_m,
        environment_snapshot.propagation_loss_db);
    const float snr_db = ToDb(received_power_w / noise_power_w);

    intercept::InterceptGateInput gate_input;
    gate_input.line_of_sight = true;
    gate_input.target_az_deg = target_az_deg;
    gate_input.target_el_deg = target_el_deg;
    gate_input.beam_az_deg = active_beam.az_deg;
    gate_input.beam_el_deg = active_beam.el_deg;
    gate_input.beam_az_width_deg = std::max(1.0f, config_.scan.az_step_deg);
    gate_input.beam_el_width_deg = std::max(1.0f, config_.scan.el_step_deg);
    gate_input.receiver_lower_hz = receiver_window.first;
    gate_input.receiver_upper_hz = receiver_window.second;
    gate_input.signal_center_hz = emitter.carrier_hz;
    gate_input.signal_bandwidth_hz = emitter.bandwidth_hz;
    gate_input.range_m = range_m;
    gate_input.max_range_m =
        boundary_result.boundary_range_m > 0.0f ? boundary_result.boundary_range_m
                                                 : 0.0f;
    gate_input.dynamic_range_margin_db =
        snr_db - config_.detection.min_detect_snr_db;
    gate_input.min_dynamic_range_margin_db =
        config_.detection.min_dynamic_range_margin_db;
    gate_input.min_frequency_overlap_ratio = 0.05f;
    gate_input.beam_guard_factor = 1.5f;

    const intercept::InterceptGateDecision gate_decision =
        intercept::InterceptGate::Evaluate(gate_input);
    const float effective_beamwidth_deg =
        std::max(1.0f, 0.5f * (gate_input.beam_az_width_deg +
                               gate_input.beam_el_width_deg));
    const float measured_az_deg =
        target_az_deg + intercept::AngleErrorModel::SampleErrorDeg(
                            snr_db, effective_beamwidth_deg, &rng_,
                            angle_error_config);
    const float measured_el_deg =
        target_el_deg + intercept::AngleErrorModel::SampleErrorDeg(
                            snr_db, effective_beamwidth_deg, &rng_,
                            angle_error_config);
    const bool is_jammed =
        effective_suppression_power_w >=
        static_cast<double>(std::max(
            0.0f, config_.suppression_model.suppression_mark_threshold_w));
    const float deception_probability =
        Clamp01(jamming_result.deception_risk *
                jamming_result.deception_weighted_overlap_ratio *
                std::max(0.0f,
                         config_.deception_model.false_alarm_probability_scale));

    internal::RawObservationRecord base_record;
    base_record.observation.observation_id = next_observation_id_++;
    base_record.observation.timestamp_s =
        static_cast<double>(input_state.cycle_index) *
        static_cast<double>(input_state.dt_sec);
    base_record.observation.aoa_az_deg = measured_az_deg;
    base_record.observation.aoa_el_deg = measured_el_deg;
    base_record.observation.rf_hz = emitter.carrier_hz;
    base_record.observation.pulse_width_s = emitter.pulse_width_s;
    base_record.observation.amplitude_db = ToDb(received_power_w);
    base_record.observation.snr_db = snr_db;
    base_record.observation.quality = ClassifyObservationQuality(snr_db, is_jammed);
    base_record.observation.is_jammed = is_jammed;

    const std::uint32_t false_alarm_cap =
        config_.deception_model.max_false_observations_per_emitter;
    if (!gate_decision.passed) {
      for (std::uint32_t fake_index = 0U; fake_index < false_alarm_cap;
           ++fake_index) {
        if (uniform_01(rng_) >= deception_probability) {
          continue;
        }
        raw_records.push_back(
            BuildDeceptionRecord(base_record, deception_probability,
                                 config_.deception_model, &rng_,
                                 &next_observation_id_));
      }
      continue;
    }

    internal::RawObservationRecord record = base_record;
    record.truth_emitter_id = emitter.emitter_id;
    record.matched_truth = true;

    const float confusion_probability =
        Clamp01(deception_probability *
                std::max(0.0f, config_.deception_model.confusion_probability_scale));
    if (uniform_01(rng_) < confusion_probability) {
      ApplyDeceptionConfusion(deception_probability, config_.deception_model, &rng_,
                              &record);
    }
    raw_records.push_back(record);

    for (std::uint32_t fake_index = 0U; fake_index < false_alarm_cap;
         ++fake_index) {
      if (uniform_01(rng_) >= deception_probability) {
        continue;
      }
      raw_records.push_back(
          BuildDeceptionRecord(record, deception_probability,
                               config_.deception_model, &rng_,
                               &next_observation_id_));
    }
  }

  const std::vector<internal::RawObservationRecord> records =
      preprocessor_.Run(raw_records, config_.preprocess);
  result.observations.reserve(records.size());
  for (std::size_t i = 0; i < records.size(); ++i) {
    result.observations.push_back(records[i].observation);
  }

  std::vector<internal::ObservationFeatureVector> features;
  features.reserve(records.size());
  for (std::size_t i = 0; i < records.size(); ++i) {
    features.push_back(
        internal::ObservationFeatureEncoder::Encode(records[i].observation,
                                                    feature_scales_));
  }

  internal::KdTreeClusterResult cluster_result =
      clusterer_.Cluster(features, config_.cluster);
  for (std::size_t i = 0; i < cluster_result.noise_indices.size(); ++i) {
    std::vector<std::size_t> singleton(1U, cluster_result.noise_indices[i]);
    cluster_result.clusters.push_back(singleton);
  }
  std::sort(cluster_result.clusters.begin(), cluster_result.clusters.end(),
            [](const std::vector<std::size_t>& lhs,
               const std::vector<std::size_t>& rhs) {
              if (lhs.empty() || rhs.empty()) {
                return lhs.size() < rhs.size();
              }
              return lhs.front() < rhs.front();
            });

  std::vector<internal::ClusterSummary> cluster_summaries;
  cluster_summaries.reserve(cluster_result.clusters.size());
  for (std::size_t i = 0; i < cluster_result.clusters.size(); ++i) {
    cluster_summaries.push_back(
        BuildClusterSummary(cluster_result.clusters[i], records, features,
                            config_.deception_model));
  }

  associator_.UpdateConfig(config_.association);
  result.emitter_hypotheses = associator_.Update(
      input_state.cycle_index, cluster_summaries, &next_hypothesis_id_);

  std::set<std::string> observed_truth_ids;
  for (std::size_t i = 0; i < records.size(); ++i) {
    common::TruthAssociationRecord association;
    association.observation_id = records[i].observation.observation_id;
    association.truth_emitter_id =
        records[i].truth_emitter_id.empty() ? "UNASSOCIATED"
                                            : records[i].truth_emitter_id;
    association.matched =
        records[i].matched_truth && !records[i].truth_emitter_id.empty();
    association.confidence = association.matched
                                 ? ComputeObservationConfidence(
                                       records[i].observation.snr_db,
                                       records[i].observation.is_jammed)
                                 : 0.1f;
    result.truth_associations.push_back(association);
    if (association.matched) {
      observed_truth_ids.insert(records[i].truth_emitter_id);
    }
  }

  for (std::size_t i = 0; i < input_state.scene_emitters.size(); ++i) {
    if (!input_state.scene_emitters[i].is_emitting) {
      continue;
    }
    if (observed_truth_ids.find(input_state.scene_emitters[i].emitter_id) !=
        observed_truth_ids.end()) {
      continue;
    }
    common::TruthAssociationRecord missed_association;
    missed_association.observation_id = 0U;
    missed_association.truth_emitter_id = input_state.scene_emitters[i].emitter_id;
    missed_association.matched = false;
    missed_association.confidence = 0.0f;
    result.truth_associations.push_back(missed_association);
  }

  return result;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
