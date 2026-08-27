/**
 * @file RirAcceptanceRecords.cpp
 * @brief RIR 验收行与天线三维增益 CSV。
 */

#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/remote_identification_radar/session/RirFeatureMeasurementTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "common/logging/AcceptanceText.h"
#include "common/logging/LogDirectory.h"
#include "common/numerics/Constants.h"
#include "common/radar/MtiMtdAcceptanceBank.h"
#include "common/radar/RadarEquations.h"
#include "remote_identification_radar/dwell/RirAntennaPatternRuntime.h"
#include "remote_identification_radar/runtime/PolarizationAcceptanceS.h"
#include "remote_identification_radar/runtime/RirAcceptanceLog.h"
#include "remote_identification_radar/tracking/RirTrackAssociator.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

#ifndef ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH
#define ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH "log/rir_antenna_pattern.csv"
#endif
#ifndef ONEQ_RIR_SCAN_PATTERN_CSV_PATH
#define ONEQ_RIR_SCAN_PATTERN_CSV_PATH "log/rir_scan_pattern.csv"
#endif

namespace remote_identification_radar {
namespace runtime {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatPairDeg;
using oneq::logging::FormatSci;
using oneq::logging::FormatVec3;
using oneq::logging::YesNo;

const char* CategoryName(int category) {
  using session::RirRecognitionCategory;
  switch (category) {
    case static_cast<int>(RirRecognitionCategory::kBallistic):
      return "弹道目标";
    case static_cast<int>(RirRecognitionCategory::kNearSpace):
      return "临近空间目标";
    case static_cast<int>(RirRecognitionCategory::kOther):
      return "其它";
    case static_cast<int>(RirRecognitionCategory::kUnknown):
      return "未知";
    case static_cast<int>(RirRecognitionCategory::kFighter):
      return "战斗机";
    case static_cast<int>(RirRecognitionCategory::kBomber):
      return "轰炸机";
    case static_cast<int>(RirRecognitionCategory::kMissile):
      return "导弹";
    default:
      return "无";
  }
}

constexpr double kLn10 = 2.302585092994046;

double ToDb(double linear) {
  if (linear <= 0.0) {
    return -999.0;
  }
  return 10.0 * std::log(linear) / kLn10;
}

double FromDb(double db) { return std::pow(10.0, db / 10.0); }

const char* TrackStatusText(tracking::RirTrackStatus status) {
  switch (status) {
    case tracking::RirTrackStatus::kConfirmed:
      return "确认";
    case tracking::RirTrackStatus::kLost:
      return "丢失";
    case tracking::RirTrackStatus::kTentative:
    default:
      return "候选";
  }
}

double HeadingDeg(double ve, double vn) {
  return oneq::common::numerics::RadToDeg(std::atan2(vn, ve));
}

std::string FormatChannelWatts(const std::array<double, oneq::common::radar::kMtiMtdChannelCount>& values) {
  std::string text = "[";
  for (std::size_t i = 0U; i < values.size(); ++i) {
    if (i != 0U) {
      text += ",";
    }
    text += FormatSci(values[i]);
  }
  text += "]";
  return text;
}

bool TryBuildAcceptanceBank(const RirDetectionAcceptInput& input,
                            oneq::common::radar::MtiMtdAcceptanceResult* bank) {
  if (bank == nullptr || !input.has_cell) {
    return false;
  }
  std::vector<oneq::common::radar::MtiMtdInterferenceTone> tones;
  tones.reserve(input.incident_links.size());
  for (const auto& link : input.incident_links) {
    if (!std::isfinite(link.doppler_shift_hz)) {
      continue;
    }
    const double power = std::isfinite(link.received_power_before_overlap_w)
                             ? link.received_power_before_overlap_w
                             : link.received_power_w;
    if (!std::isfinite(power) || power < 0.0) {
      continue;
    }
    oneq::common::radar::MtiMtdInterferenceTone tone;
    tone.doppler_hz = link.doppler_shift_hz;
    tone.power_w = power;
    tones.push_back(tone);
  }
  oneq::common::radar::MtiMtdAcceptanceInput bank_input;
  bank_input.echo_power_w = input.cell.echo_power_w;
  bank_input.thermal_noise_power_w = input.cell.thermal_noise_power_w;
  bank_input.clutter_power_w = input.cell.clutter_power_w;
  bank_input.two_way_doppler_shift_hz = input.cell.two_way_doppler_shift_hz;
  bank_input.prf_hz = input.prf_hz;
  bank_input.center_frequency_hz = input.center_frequency_hz;
  if (!tones.empty()) {
    bank_input.tones = tones.data();
    bank_input.tone_count = tones.size();
  }
  return oneq::common::radar::TryResolveMtiMtdAcceptanceBank(bank_input, bank);
}

double SumChannelWatts(const std::array<double, oneq::common::radar::kMtiMtdChannelCount>& values) {
  double sum = 0.0;
  for (double value : values) {
    sum += value;
  }
  return sum;
}

double SumIncidentJamWatts(const RirDetectionAcceptInput& input) {
  double sum = 0.0;
  for (const auto& link : input.incident_links) {
    if (!std::isfinite(link.doppler_shift_hz)) {
      continue;
    }
    const double power = std::isfinite(link.received_power_before_overlap_w)
                             ? link.received_power_before_overlap_w
                             : link.received_power_w;
    if (std::isfinite(power) && power > 0.0) {
      sum += power;
    }
  }
  return sum;
}

void Emit(float sim_time_sec, std::uint32_t cycle, const char* item, const std::string& content) {
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, item, content);
}

void EmitOrNone(float sim_time_sec, std::uint32_t cycle, const char* item, const std::string& prefix,
                bool ok, const std::string& value) {
  Emit(sim_time_sec, cycle, item, prefix + (ok ? value : std::string("无")));
}

/** 航迹 ENU → 验收斜距/方位/俯仰；无效位置保持「无」。 */
std::string FormatTrackLookPolar(const tracking::RirTrackState& track) {
  float range_m = 0.0f;
  float az_deg = 0.0f;
  float el_deg = 0.0f;
  if (!TryLookPolarFromEnuM(track.position.x(), track.position.y(), track.position.z(), &range_m,
                            &az_deg, &el_deg)) {
    return "斜距=无 方位/俯仰=无";
  }
  return "斜距=" + FormatF(range_m, 1) + "m 方位/俯仰=" + FormatPairDeg(az_deg, el_deg, 3) + "°";
}

}  // namespace

bool TryLookPolarFromEnuM(float east_m, float north_m, float up_m, float* range_m, float* az_deg,
                          float* el_deg) {
  if (range_m == nullptr || az_deg == nullptr || el_deg == nullptr) {
    return false;
  }
  if (!std::isfinite(east_m) || !std::isfinite(north_m) || !std::isfinite(up_m)) {
    return false;
  }
  const float range = std::sqrt(east_m * east_m + north_m * north_m + up_m * up_m);
  if (!(range > 0.1f)) {
    return false;
  }
  const float range_hypot = std::sqrt(east_m * east_m + north_m * north_m);
  *range_m = range;
  *az_deg = oneq::common::numerics::RadToDeg(std::atan2(north_m, east_m));
  *el_deg = oneq::common::numerics::RadToDeg(std::atan2(up_m, range_hypot));
  return true;
}

std::string ResolveRirAntennaPatternCsvPath() {
#if defined(_MSC_VER)
  char* buffer = nullptr;
  if (_dupenv_s(&buffer, nullptr, "ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH") == 0 && buffer != nullptr) {
    std::string result(buffer);
    std::free(buffer);
    if (!result.empty()) {
      return result;
    }
  }
#else
  const char* env = std::getenv("ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH");
  if (env != nullptr && *env != '\0') {
    return std::string(env);
  }
#endif
  return std::string(ONEQ_RIR_ANTENNA_PATTERN_CSV_PATH);
}

std::string ResolveRirScanPatternCsvPath() {
#if defined(_MSC_VER)
  char* buffer = nullptr;
  if (_dupenv_s(&buffer, nullptr, "ONEQ_RIR_SCAN_PATTERN_CSV_PATH") == 0 && buffer != nullptr) {
    std::string result(buffer);
    std::free(buffer);
    if (!result.empty()) {
      return result;
    }
  }
#else
  const char* env = std::getenv("ONEQ_RIR_SCAN_PATTERN_CSV_PATH");
  if (env != nullptr && *env != '\0') {
    return std::string(env);
  }
#endif
  return std::string(ONEQ_RIR_SCAN_PATTERN_CSV_PATH);
}

void WriteRirAntennaPatternSummary(float sim_time_sec, std::uint32_t cycle, float peak_gain_dbi,
                                   float bw_az_deg, float bw_el_deg, const std::string& csv_path) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::string content = "三维增益文件=" + csv_path;
  content += " 峰值增益=" + FormatF(static_cast<double>(peak_gain_dbi), 2) + "dBi";
  content += " 波束宽度az/el=" + FormatPairDeg(bw_az_deg, bw_el_deg, 3) + "°";
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "天线方向图仿真", content);
}

void WriteRirDetectionChain(const RirDetectionAcceptInput& input) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const float t = input.sim_time_sec;
  const std::uint32_t cycle = input.cycle;
  const std::string id = "目标ID=" + std::to_string(input.target_id);
  const std::string id_sp = id + " ";
  const double thermal = input.has_cell ? input.cell.thermal_noise_power_w : 0.0;
  const double jam = input.has_cell ? input.cell.interference_power_w : 0.0;
  const double clutter = input.has_cell ? input.cell.clutter_power_w : 0.0;
  const double pc = input.has_cell ? input.cell.pulse_compression_gain : 1.0;
  const std::uint32_t pulses = input.has_cell ? input.cell.effective_pulse_count : 1U;
  const double pc_db = ToDb(pc);
  const double coherent_db = ToDb(static_cast<double>(std::max(1U, pulses)));
  const double noise_db = static_cast<double>(input.gains.noise_processing_gain_db);
  const double clutter_db = static_cast<double>(input.gains.clutter_suppression_gain_db);
  const double pc_noise = thermal * std::max(pc, 0.0);
  const double mti_residual = clutter_db <= 0.0 ? clutter : clutter / FromDb(clutter_db);
  oneq::common::radar::MtiMtdAcceptanceResult bank;
  const bool has_bank = TryBuildAcceptanceBank(input, &bank);

  std::string echo = id;
  echo += " 斜距=" + FormatF(input.range_m, 1) + "m";
  echo += " SNR=" + FormatF(input.snr_db, 3) + "dB";
  echo += " 回波功率=" + FormatF(input.echo_power_dbw, 3) + "dBW";
  echo += " 热噪声=" + FormatSci(thermal) + "W";
  echo += " 干扰功率=" + FormatSci(jam) + "W";
  echo += " 杂波功率=" + FormatSci(clutter) + "W";
  Emit(t, cycle, "经处理后雷达回波信号", echo);
  Emit(t, cycle, "回波功率计算", echo);

  Emit(t, cycle, "接收机噪声功率",
       id_sp + "热噪声功率=" + FormatSci(thermal) + "W SNR=" + FormatF(input.snr_db, 3) + "dB");

  Emit(t, cycle, "脉压增益", id_sp + FormatF(pc_db, 3) + "dB");
  Emit(t, cycle, "相干积累增益",
       id_sp + FormatF(coherent_db, 3) + "dB N=" + std::to_string(pulses));
  EmitOrNone(t, cycle, "MTI增益", id_sp, has_bank,
             has_bank ? FormatF(bank.mti_gain_db, 3) + "dB" : std::string());
  EmitOrNone(t, cycle, "MTD增益", id_sp, has_bank,
             has_bank ? FormatF(bank.mtd_gain_db, 3) + "dB" : std::string());

  Emit(t, cycle, "脉冲压缩后的噪声功率", id_sp + FormatSci(pc_noise) + "W");
  EmitOrNone(t, cycle, "各多普勒滤波器通道噪声功率", id_sp, has_bank,
             has_bank ? FormatChannelWatts(bank.noise_w) + "W" : std::string());
  EmitOrNone(t, cycle, "MTD等效噪声功率", id_sp, has_bank,
             has_bank ? FormatSci(bank.mtd_equivalent_noise_w) + "W" : std::string());
  Emit(t, cycle, "噪声总增益", id_sp + FormatF(pc_db + noise_db, 3) + "dB");

  Emit(t, cycle, "MTI处理后杂波剩余功率",
       id_sp + FormatSci(has_bank ? bank.mti_residual_clutter_w : mti_residual) + "W");
  EmitOrNone(t, cycle, "MTD各多普勒通道杂波剩余功率分布", id_sp, has_bank,
             has_bank ? FormatChannelWatts(bank.clutter_w) + "W" : std::string());
  double clutter_ratio_db = 0.0;
  const bool has_clutter_ratio =
      has_bank && oneq::common::radar::TryAcceptancePowerRatioDb(
                      clutter, SumChannelWatts(bank.clutter_w), &clutter_ratio_db);
  EmitOrNone(t, cycle, "MTD处理后的杂波抑制比", id_sp, has_clutter_ratio,
             FormatF(clutter_ratio_db, 3) + "dB");
  Emit(t, cycle, "总杂波抑制增益", id_sp + FormatF(clutter_db, 3) + "dB");

  const bool has_jam = has_bank && bank.has_jam_channels;
  const double jam_in = SumIncidentJamWatts(input);
  double mtd_jam_ratio_db = 0.0;
  double total_jam_ratio_db = 0.0;
  const bool has_mtd_jam =
      has_jam && oneq::common::radar::TryAcceptancePowerRatioDb(
                     jam_in, SumChannelWatts(bank.jam_w), &mtd_jam_ratio_db);
  const bool has_total_jam =
      has_jam && oneq::common::radar::TryAcceptancePowerRatioDb(jam_in, bank.mti_residual_jam_w,
                                                               &total_jam_ratio_db);
  EmitOrNone(t, cycle, "MTI处理后干扰剩余功率", id_sp, has_jam,
             has_jam ? FormatSci(bank.mti_residual_jam_w) + "W" : std::string());
  EmitOrNone(t, cycle, "MTD各多普勒通道干扰功率分布", id_sp, has_jam,
             has_jam ? FormatChannelWatts(bank.jam_w) + "W" : std::string());
  EmitOrNone(t, cycle, "MTD处理后的干扰总抑制比", id_sp, has_mtd_jam,
             FormatF(mtd_jam_ratio_db, 3) + "dB");
  EmitOrNone(t, cycle, "总干扰抑制增益", id_sp, has_total_jam,
             FormatF(total_jam_ratio_db, 3) + "dB");

  bool has_threshold = false;
  double threshold = 0.0;
  if (input.has_cell && std::isfinite(input.cfar_pfa) && input.cfar_pfa > 0.0 &&
      input.cfar_pfa < 1.0) {
    threshold = oneq::common::radar::RadarEquations::ComputeThreshold(
        input.cfar_pfa, static_cast<int>(std::max(1U, pulses)));
    has_threshold = std::isfinite(threshold);
  }
  EmitOrNone(t, cycle, "统计检测门限", id_sp, has_threshold, FormatF(threshold, 6));
  Emit(t, cycle, "统计检测结果", id_sp + YesNo(input.detected));
  Emit(t, cycle, "统计检测概率", id_sp + FormatF(input.pd, 5));

  std::string rcs = id;
  rcs += " 本周期RCS=" + FormatF(input.rcs_m2, 3) + "m²";
  rcs += " 斜距=" + FormatF(input.range_m, 1) + "m";
  rcs += " 方位/俯仰=" + FormatPairDeg(input.look_az_deg, input.look_el_deg, 3) + "°";
  Emit(t, cycle, "RCS实时探测", rcs);
}

void WriteRirInterferenceLinks(
    float sim_time_sec, std::uint32_t cycle,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& links) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  if (links.empty()) {
    RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "干扰功率计算", "干扰源=无 到达雷达功率=0.000W");
    return;
  }
  for (const oneq::electromagnetics::RfIncidentLinkResult& link : links) {
    std::string content = "干扰源=平台";
    content += std::to_string(link.identity.platform_id);
    content += "/设备";
    content += std::to_string(link.identity.equipment_id);
    content += "/发射";
    content += std::to_string(link.identity.emission_id);
    content += " 到达雷达功率=" + FormatSci(link.received_power_w) + "W";
    content += " 路径=" + FormatF(link.path_length_m, 1) + "m";
    content += " 时域/频域重叠=" + FormatF(link.time_overlap_fraction, 4) + "/" +
               FormatF(link.frequency_overlap_fraction, 4);
    RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "干扰功率计算", content);
  }
}

void WriteRirSearchDetections(float sim_time_sec, std::uint32_t cycle, float beam_az_deg,
                              float beam_el_deg,
                              const config::RirAzimuthElevationLimitsDeg& search_volume_deg,
                              const config::RirAzimuthElevationDeg& scan_center_deg,
                              const std::string& found_targets) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const std::string pointing =
      "方位/俯仰=" + FormatPairDeg(beam_az_deg, beam_el_deg, 3) + "°";
  // 2026-08-22 甲方批注「设定方位俯仰进行扫描」：搜索集合按可扫描体积裁剪
  // （检测候选与该角域同源）；行内给出中心与角域上下界。
  std::string sector = "搜索角域=中心(az" + FormatF(scan_center_deg.az_deg, 1) + ",el" +
                       FormatF(scan_center_deg.el_deg, 1) + ")°";
  sector += " az[" + FormatF(search_volume_deg.az_min_deg, 1) + "," +
            FormatF(search_volume_deg.az_max_deg, 1) + "]°相对中心";
  sector += " el[" + FormatF(search_volume_deg.el_min_deg, 1) + "," +
            FormatF(search_volume_deg.el_max_deg, 1) + "]°";
  const std::string detections = "搜到目标=[" + found_targets + "]";
  Emit(sim_time_sec, cycle, "本周期方位俯仰指向", pointing);
  Emit(sim_time_sec, cycle, "检测量测信息", detections);
  Emit(sim_time_sec, cycle, "指定空域搜索", pointing + " " + sector + " " + detections);
}

void WriteRirAssociation(float sim_time_sec, std::uint32_t cycle,
                         const tracking::RirAssociationResult& association,
                         const oneq::coordinate::LlaPositionDegM& platform_lla,
                         double gate_threshold) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::string content = "新量测=" + std::to_string(association.measurements.size());
  content += " 配到已有航迹=" + std::to_string(association.matches.size());
  content += " 未配上=" + std::to_string(association.missed_track_keys.size());
  // 条16：马氏代价写明口径与波门（紧跟计数，避免括号误挂到量测明细尾部）。
  content += "（马氏距离²=νᵀS⁻¹ν，无量纲，波门=" + FormatF(gate_threshold, 1) + "）";
  // 评审 2026-08-26 条15：逐量测明细（目标ID、位置 LLA、检测概率、关联结果——
  // 完整描述不简写）。
  if (!association.measurements.empty()) {
    content += " 量测明细：";
    for (std::size_t i = 0U; i < association.measurements.size(); ++i) {
      if (i != 0U) {
        content += "；";
      }
      const tracking::RirTrackMeasurement& measurement = association.measurements[i];
      content += "量测" + std::to_string(measurement.source_index);
      content += "(目标ID=" + std::to_string(measurement.external_target_id);
      oneq::coordinate::EcefPositionM measurement_ecef;
      oneq::coordinate::LlaPositionDegM measurement_lla;
      if (oneq::coordinate::TryEnuToEcef(
              oneq::coordinate::EnuPositionM(measurement.position.x(), measurement.position.y(),
                                             measurement.position.z()),
              platform_lla, &measurement_ecef) &&
          oneq::coordinate::TryEcefToLla(measurement_ecef, &measurement_lla)) {
        content += ",位置LLA=(" + FormatF(measurement_lla.latitude_deg, 6) + "," +
                   FormatF(measurement_lla.longitude_deg, 6) + "," +
                   FormatF(measurement_lla.altitude_m, 1) + ")m";
      } else {
        content += ",位置LLA=无";
      }
      content += ",检测概率=" + FormatF(measurement.detection_pd, 4) + ")";
      content += measurement.matched_existing_track
                     ? " 关联结果=配到已有航迹" + std::to_string(measurement.association_key)
                     : " 关联结果=未配到任何已有航迹";
    }
  }
  if (!association.matches.empty()) {
    content += " 命中：";
    for (std::size_t i = 0; i < association.matches.size(); ++i) {
      if (i != 0U) {
        content += "；";
      }
      const tracking::RirAssociationMatch& match = association.matches[i];
      content += "航迹" + std::to_string(match.association_key) + "←量测" +
                 std::to_string(match.source_index) + " 马氏距离²=" +
                 FormatF(match.cost, 4);
    }
  }
  Emit(sim_time_sec, cycle, "航迹关联", content);
  Emit(sim_time_sec, cycle, "更新后的航迹集合", content);
}

void WriteRirClusterCount(float sim_time_sec, std::uint32_t cycle,
                          const std::vector<tracking::RirTrackState>& tracks) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 甲方 2026-08-22 批注「要输出集群目标数量（知道能打出几枚即可）」：
  // 集群目标数量 = 确认航迹数（身份去重后的在跟目标数，非检测条数——
  // 「检测条数不得顶替集群规模」原裁定由本批注按计数口径覆盖）。同时给出
  // 待确认/丢失计数作上下文；无确认航迹时如实写 0。
  std::size_t confirmed = 0U;
  std::size_t tentative = 0U;
  std::size_t lost = 0U;
  for (const tracking::RirTrackState& track : tracks) {
    switch (track.status) {
      case tracking::RirTrackStatus::kConfirmed:
        ++confirmed;
        break;
      case tracking::RirTrackStatus::kTentative:
        ++tentative;
        break;
      case tracking::RirTrackStatus::kLost:
        ++lost;
        break;
    }
  }
  std::string content = "集群目标数量=" + std::to_string(confirmed);
  content += " 在跟=" + std::to_string(confirmed + tentative);
  content += " 待确认=" + std::to_string(tentative);
  content += " 丢失=" + std::to_string(lost);
  Emit(sim_time_sec, cycle, "集群目标识别", content);
}

void WriteRirTrackAndId(float sim_time_sec, std::uint32_t cycle, const tracking::RirTrackState& track,
                        const session::RirRecognitionResult* result,
                        const session::RirFeatureMeasurementRecord* features,
                        const std::vector<session::RirPolarizationRcsSample>* polarization_samples,
                        bool has_truth, double category_accuracy,
                        const std::vector<float>* imm_weights,
                        const oneq::coordinate::EcefPositionM& platform_ecef) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const double heading = HeadingDeg(track.velocity.x(), track.velocity.y());
  const double dt = 1.0;
  const int category = result != nullptr ? static_cast<int>(result->target_category) : -1;
  // 评审 2026-08-26 条19/20：不输出「大类枚举N(名)」外壳，直接给中文名；未识别
  // （result 缺失/类别未知且无识别结果）写「未识别」。
  const std::string category_text = category < 0 ? std::string("未识别")
                                                 : std::string(CategoryName(category));
  const std::string polar = FormatTrackLookPolar(track);
  // 评审 2026-08-26 条17：ENU 字段替换为 ECEF/LLA（平台 ECEF 为雷达局部 ENU 的
  // 绝对锚点；锚点或换算失败时如实写无）。速度/加速度按 ENU→ECEF 旋转矩阵换算。
  oneq::coordinate::LlaPositionDegM platform_lla;
  oneq::coordinate::EcefPositionM pos_ecef;
  oneq::coordinate::LlaPositionDegM pos_lla;
  oneq::coordinate::EcefPositionM next_ecef;
  oneq::coordinate::LlaPositionDegM next_lla;
  oneq::coordinate::EcefVelocityMps vel_ecef;
  oneq::coordinate::EcefVelocityMps acc_ecef;
  bool have_frame = false;
  if (oneq::coordinate::TryEcefToLla(platform_ecef, &platform_lla)) {
    const oneq::coordinate::EnuPositionM pos_enu(track.position.x(), track.position.y(),
                                                 track.position.z());
    const oneq::coordinate::EnuPositionM next_enu(
        track.position.x() + track.velocity.x() * dt, track.position.y() + track.velocity.y() * dt,
        track.position.z() + track.velocity.z() * dt);
    const oneq::coordinate::EnuVelocityMps vel_enu(track.velocity.x(), track.velocity.y(),
                                                   track.velocity.z());
    const oneq::coordinate::EnuVelocityMps acc_enu(track.acceleration.x(), track.acceleration.y(),
                                                   track.acceleration.z());
    have_frame =
        oneq::coordinate::TryEnuToEcef(pos_enu, platform_lla, &pos_ecef) &&
        oneq::coordinate::TryEcefToLla(pos_ecef, &pos_lla) &&
        oneq::coordinate::TryEnuToEcef(next_enu, platform_lla, &next_ecef) &&
        oneq::coordinate::TryEcefToLla(next_ecef, &next_lla) &&
        oneq::coordinate::TryEnuToEcefVelocity(vel_enu, platform_lla, &vel_ecef) &&
        oneq::coordinate::TryEnuToEcefVelocity(acc_enu, platform_lla, &acc_ecef);
  }
  const std::string pos_ecef_text =
      have_frame ? FormatVec3(pos_ecef.x_m, pos_ecef.y_m, pos_ecef.z_m, 1) : std::string("无");
  const std::string pos_lla_text =
      have_frame
          ? FormatVec3(pos_lla.latitude_deg, pos_lla.longitude_deg, pos_lla.altitude_m, 6)
          : std::string("无");
  const std::string next_ecef_text =
      have_frame ? FormatVec3(next_ecef.x_m, next_ecef.y_m, next_ecef.z_m, 1) : std::string("无");
  const std::string next_lla_text =
      have_frame
          ? FormatVec3(next_lla.latitude_deg, next_lla.longitude_deg, next_lla.altitude_m, 6)
          : std::string("无");
  const std::string vel_ecef_text =
      have_frame ? FormatVec3(vel_ecef.x_mps, vel_ecef.y_mps, vel_ecef.z_mps, 3) : std::string("无");
  const std::string acc_ecef_text =
      have_frame ? FormatVec3(acc_ecef.x_mps, acc_ecef.y_mps, acc_ecef.z_mps, 4) : std::string("无");
  std::string measure = "目标ID=" + std::to_string(track.external_target_id);
  measure += " " + polar;
  measure += " 高度=" + FormatF(track.position.z(), 1) + "m";
  measure += " 速度=" + FormatF(track.speed, 3) + "m/s";
  measure += " 航向=" + FormatF(heading, 2) + "°";
  measure += " 加速度=" + FormatVec3(track.acceleration.x(), track.acceleration.y(),
                                     track.acceleration.z(), 4) +
             "m/s²";
  measure += " 飞机类型=" + category_text;
  Emit(sim_time_sec, cycle, "目标测量角度与距离", measure);
  Emit(sim_time_sec, cycle, "角度和距离测量", measure);

  std::string reentry = "航迹=" + std::to_string(track.association_key);
  reentry += " 目标ID=" + std::to_string(track.external_target_id);
  reentry += std::string(" 状态=") + TrackStatusText(track.status);
  reentry += " " + polar;
  reentry += " 高度=" + FormatF(track.position.z(), 1) + "m";
  reentry += " 速度=" + FormatF(track.speed, 3) + "m/s";
  reentry += " 航向=" + FormatF(heading, 2) + "°";
  reentry += " 加速度ECEF=" + acc_ecef_text + "m/s²";
  Emit(sim_time_sec, cycle, "典型/再入目标跟踪", reentry);
  Emit(sim_time_sec, cycle, "多目标跟踪", reentry);

  const tracking::RirStateCovariance& cov = track.gaussian_state.covariance;
  std::string filter = "航迹=" + std::to_string(track.association_key);
  filter += " 当前ECEF=" + pos_ecef_text + "m";
  filter += " 当前LLA=" + pos_lla_text;
  filter += " 下一时刻预测ECEF=" + next_ecef_text + "m";
  filter += " 下一时刻预测LLA=" + next_lla_text;
  filter += " 速度ECEF=" + vel_ecef_text + "m/s";
  filter += " 加速度ECEF=" + acc_ecef_text + "m/s²";
  filter += " 协方差迹=" + FormatF(static_cast<double>(track.EstimationUncertaintyTrace()), 2);
  filter += " 完整协方差=[";
  for (int row = 0; row < 6; ++row) {
    if (row != 0) {
      filter += ";";
    }
    for (int col = 0; col < 6; ++col) {
      if (col != 0) {
        filter += ",";
      }
      filter += FormatF(static_cast<double>(cov(row, col)), 6);
    }
  }
  filter += "]";
  Emit(sim_time_sec, cycle, "目标位置速度加速度估计",
       "航迹=" + std::to_string(track.association_key) + " 当前ECEF=" + pos_ecef_text +
           "m 当前LLA=" + pos_lla_text + " 下一时刻预测ECEF=" + next_ecef_text +
           "m 下一时刻预测LLA=" + next_lla_text + " 速度ECEF=" + vel_ecef_text +
           "m/s 加速度ECEF=" + acc_ecef_text + "m/s²");
  const std::string cov_text = filter.substr(filter.find("完整协方差="));
  Emit(sim_time_sec, cycle, "目标状态协方差",
       "航迹=" + std::to_string(track.association_key) + " " + cov_text);
  Emit(sim_time_sec, cycle, "跟踪滤波", filter);
  if (imm_weights != nullptr && !imm_weights->empty()) {
    std::string weights = "航迹=" + std::to_string(track.association_key) + " 权重=[";
    for (std::size_t i = 0U; i < imm_weights->size(); ++i) {
      if (i != 0U) {
        weights += ",";
      }
      weights += FormatF(static_cast<double>((*imm_weights)[i]), 3);
    }
    weights += "]";
    Emit(sim_time_sec, cycle, "IMM模型权重", weights);
  } else {
    Emit(sim_time_sec, cycle, "IMM模型权重",
         "航迹=" + std::to_string(track.association_key) + " 无");
  }

  std::string id_text = "航迹=" + std::to_string(track.association_key);
  id_text += " 目标ID=" + std::to_string(track.external_target_id);
  if (result != nullptr) {
    id_text += " 状态=" + std::to_string(static_cast<int>(result->state));
    id_text += " 大类=" + category_text;
    id_text += " 型号=" + (result->target_model.empty() ? std::string("无") : result->target_model);
    id_text += " 置信度=" + FormatF(static_cast<double>(result->confidence), 4);
  } else {
    id_text += " 无";
  }
  Emit(sim_time_sec, cycle, "独立目标识别器结论", id_text);

  if (features != nullptr) {
    const auto& motion = features->features.motion;
    std::string motion_text = "航迹=" + std::to_string(track.association_key);
    motion_text += " 速度=" + FormatF(motion.speed_m_per_s, 3) + "m/s";
    motion_text += " 高度=" + FormatF(motion.altitude_m, 1) + "m";
    // 评审 2026-08-26 条18：补斜距与方位/俯仰（features 自带视线几何量）。
    motion_text += " 斜距=" + FormatF(features->range_m, 1) + "m";
    motion_text += " 方位/俯仰=" + FormatPairDeg(features->look_az_deg, features->look_el_deg, 3) +
                   "°";
    motion_text += std::string(" 近似直线=") + YesNo(motion.is_straight);
    motion_text += " 目标类别=" + category_text;
    Emit(sim_time_sec, cycle, "运动特征处理", motion_text);
    Emit(sim_time_sec, cycle, "目标类别", motion_text);

    const auto& rcs = features->features.rcs;
    std::string rcs_text = "航迹=" + std::to_string(track.association_key);
    rcs_text += " RCS均值/标准差=" + FormatF(rcs.mean_dbsm, 3) + "/" + FormatF(rcs.std_db, 3) + "dBsm";
    rcs_text += " 目标类别=" + category_text;
    Emit(sim_time_sec, cycle, "RCS统计特征处理", rcs_text);

    const auto& pol = features->features.polarization;
    std::string pol_text = "航迹=" + std::to_string(track.association_key);
    pol_text += " 极化差/相对/和=" +
                FormatVec3(pol.energy_difference_db, pol.relative_difference_db, pol.energy_sum_db, 3);
    pol_text += " 目标类别=" + category_text;
    pol_text += " 置信度=" + FormatF(result != nullptr ? result->confidence : 0.0, 4);
    if (has_truth) {
      pol_text += " 正确识别率=" + FormatF(category_accuracy, 4);
    }
    Emit(sim_time_sec, cycle, "极化特征解算", pol_text);
    Emit(sim_time_sec, cycle, "目标类型",
         "航迹=" + std::to_string(track.association_key) + " 目标类别=" + category_text +
             " 置信度=" + FormatF(result != nullptr ? result->confidence : 0.0, 4));
    PolarizationAcceptanceSResult derived;
    const bool has_s =
        polarization_samples != nullptr &&
        TryResolvePolarizationAcceptanceS(*polarization_samples, features->look_az_deg,
                                          features->look_el_deg, &derived);
    const std::string track_id = "航迹=" + std::to_string(track.association_key) + " ";
    EmitOrNone(sim_time_sec, cycle, "功率迹", track_id, has_s,
               has_s ? FormatSci(derived.span) : std::string());
    EmitOrNone(sim_time_sec, cycle, "极化散射矩阵行列式", track_id, has_s,
               has_s ? FormatSci(derived.abs_det) : std::string());
    EmitOrNone(sim_time_sec, cycle, "去极化系数", track_id, has_s,
               has_s ? FormatF(derived.depolarization, 4) : std::string());
    EmitOrNone(sim_time_sec, cycle, "本征极化方向角", track_id, has_s,
               has_s ? FormatF(derived.psi_deg, 3) + "°" : std::string());
    EmitOrNone(sim_time_sec, cycle, "本征极化椭圆率", track_id, has_s,
               has_s ? FormatF(derived.tau_deg, 3) + "°" : std::string());

    const auto& rp = features->features.range_profile;
    std::string rp_text = "航迹=" + std::to_string(track.association_key);
    rp_text += " 长度=" + FormatF(rp.length_m, 3) + "m";
    rp_text += " 峰数=" + std::to_string(rp.peak_count);
    rp_text += " 能量集中=" + FormatF(rp.peak_energy_concentration, 3);
    rp_text += " 分辨率=" + FormatF(rp.resolution_m, 3) + "m";
    const std::string rp_id = "航迹=" + std::to_string(track.association_key) + " ";
    Emit(sim_time_sec, cycle, "散射中心和轮廓特征",
         rp_id + "长度=" + FormatF(rp.length_m, 3) + "m 峰数=" + std::to_string(rp.peak_count) +
             " 能量集中=" + FormatF(rp.peak_energy_concentration, 3) +
             " 分辨率=" + FormatF(rp.resolution_m, 3) + "m");
    Emit(sim_time_sec, cycle, "识别类别",
         rp_id + "识别类型=" + category_text +
             " 置信度=" + FormatF(result != nullptr ? result->confidence : 0.0, 4));
    Emit(sim_time_sec, cycle, "宽带一维像特征解算", rp_text +
                                                         " 识别类型=" + category_text + " 置信度=" +
                                                         FormatF(result != nullptr ? result->confidence : 0.0, 4));
  }
}

// 评审 2026-08-26 条21：调度只应有搜索/跟踪两类事件（「识别」不是独立驻留模式，
// 识别在 kIdentify 模式内顺带执行），识别计数与列表段删除。
void WriteRirSchedule(float sim_time_sec, std::uint32_t cycle, std::uint32_t planned,
                      std::uint32_t executed, float budget_sec, float consumed_sec,
                      std::uint32_t search_count, std::uint32_t track_count) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::string content = "计划驻留/实际执行=" + std::to_string(planned) + "/" + std::to_string(executed);
  content += " 搜索=" + std::to_string(search_count);
  content += " 跟踪=" + std::to_string(track_count);
  content += " 预算/已耗时=" + FormatF(budget_sec, 3) + "/" + FormatF(consumed_sec, 3) + "s";
  const std::string events = "[搜索×" + std::to_string(search_count) + ",跟踪×" +
                             std::to_string(track_count) + "]";
  Emit(sim_time_sec, cycle, "各类事件的实际执行列表", events);
  Emit(sim_time_sec, cycle, "扫描调度信息", content);
  Emit(sim_time_sec, cycle, "调度策略", content);
}

void WriteRirOncePerSession(float sim_time_sec, std::uint32_t cycle) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  static bool written = false;
  if (written) {
    return;
  }
  written = true;
  // 评审 2026-08-26 条22（方案B）：库内不做墙钟计时，真实初始化/加载耗时在示例层
  // integration_events.log 的同名验收项（模块=RIR）。
  Emit(sim_time_sec, cycle, "初始化时间",
       "见integration_events.log[验收项：初始化时间]（模块=RIR）");
  Emit(sim_time_sec, cycle, "单步执行时间", "暂无");
  Emit(sim_time_sec, cycle, "单个模型加载时间",
       "见integration_events.log[验收项：单个模型加载时间]（模块=RIR）");
  Emit(sim_time_sec, cycle, "多模型并行加载",
       "见integration_events.log[验收项：多模型并行加载]（模块=RIR）");
  Emit(sim_time_sec, cycle, "典型场景和总仿真次数", "场景=本会话 场景数=1 总仿真周期=结束时回写");
}

void WriteRirCycleRunCount(float sim_time_sec, std::uint32_t cycle) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "连续运行次数",
                      "本会话已运行周期=" + std::to_string(cycle) + " 状态=正常");
}

void WriteRirBeamScan(float sim_time_sec, std::uint32_t cycle,
                      const std::vector<oneq::common::radar::AzimuthElevationDeg>& pattern,
                      float az_deg, float el_deg, bool designate) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const std::string csv_path = ResolveRirScanPatternCsvPath();
  const bool wrote = TryExportRirScanPatternCsv(pattern, csv_path.c_str());
  std::string scan = "波位总数=" + std::to_string(pattern.size());
  scan += " 本周期驻留中心方位/俯仰=" + FormatPairDeg(az_deg, el_deg, 3) + "°";
  scan += designate ? " 模式=指定" : " 模式=扫描";
  Emit(sim_time_sec, cycle, "波束扫描", scan);
  if (wrote && !pattern.empty()) {
    const std::uint64_t zero_based =
        cycle > 0U ? static_cast<std::uint64_t>(cycle - 1U) : 0U;
    const std::size_t index =
        static_cast<std::size_t>(zero_based % static_cast<std::uint64_t>(pattern.size()));
    const std::size_t next = (index + 1U) % pattern.size();
    std::string table = "文件=" + csv_path;
    table += " 本周期序号=" + std::to_string(index);
    table += " 下一波位=" + FormatPairDeg(pattern[next].az_deg, pattern[next].el_deg, 3) + "°";
    Emit(sim_time_sec, cycle, "波位排列表", table);
    Emit(sim_time_sec, cycle, "扫描轨迹序列", table);
  } else {
    Emit(sim_time_sec, cycle, "波位排列表", "无");
    Emit(sim_time_sec, cycle, "扫描轨迹序列", "无");
  }
}

bool TryExportRirAntennaPatternCsv(const config::hardware::RirAntennaConfig& antenna,
                                   const char* path) {
  const std::string out_path = (path != nullptr && *path != '\0') ? std::string(path)
                                                                  : ResolveRirAntennaPatternCsvPath();
  oneq::logging::EnsureParentDirectory(out_path.c_str());
  std::ofstream out(out_path.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
  if (!out.is_open()) {
    return false;
  }
  out << "az_off_deg,el_off_deg,gain_dbi\n";
  dwell::RirAntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = antenna.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = antenna.nominal_el_beamwidth_deg;
  config::RirAzimuthElevationDeg pointing(0.0f, 0.0f);
  // 评审 2026-08-26 条14：网格随波束宽度自适应（主瓣 ≥16 点、前几个副瓣可辨；
  // 旧 2° 步进下 4° 主瓣只有 2×2 点，主副瓣形状都画不出）。范围取 ±5·波束宽
  // （封顶 ±90°），覆盖主瓣 + 数个副瓣环；主瓣外由 sinc² 连续延拓 + 副瓣电平
  // 钳制产生起伏结构（见 AntennaPatternRuntime.h）。
  const float bw_az = std::max(0.5f, antenna.nominal_az_beamwidth_deg);
  const float bw_el = std::max(0.5f, antenna.nominal_el_beamwidth_deg);
  const float step = std::max(0.05f, std::min(bw_az, bw_el) / 20.0f);
  const float span_az = std::min(90.0f, 5.0f * bw_az);
  const float span_el = std::min(90.0f, 5.0f * bw_el);
  for (float az = -span_az; az <= span_az + 0.5f * step; az += step) {
    for (float el = -span_el; el <= span_el + 0.5f * step; el += step) {
      dwell::RirAntennaLookOffsetDeg offset;
      offset.delta_az_deg = az;
      offset.delta_el_deg = el;
      const dwell::RirAntennaPatternSample sample = dwell::RirEvaluateAntennaPattern(
          antenna.main_beam_gain_db, antenna.pattern, beamwidth, offset, pointing,
          antenna.antenna_length_m, antenna.antenna_width_m, 0.0f);
      out << FormatF(az, 2) << "," << FormatF(el, 2) << "," << FormatF(sample.gain_dbi, 3) << "\n";
    }
  }
  return true;
}

bool TryExportRirScanPatternCsv(
    const std::vector<oneq::common::radar::AzimuthElevationDeg>& pattern, const char* path) {
  const std::string out_path = (path != nullptr && *path != '\0') ? std::string(path)
                                                                  : ResolveRirScanPatternCsvPath();
  oneq::logging::EnsureParentDirectory(out_path.c_str());
  std::ofstream out(out_path.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
  if (!out.is_open()) {
    return false;
  }
  out << "index,az_deg,el_deg\n";
  for (std::size_t i = 0U; i < pattern.size(); ++i) {
    out << i << "," << FormatF(pattern[i].az_deg, 3) << "," << FormatF(pattern[i].el_deg, 3)
        << "\n";
  }
  return true;
}

}  // namespace runtime
}  // namespace remote_identification_radar
