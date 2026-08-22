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

#include "1q/remote_identification_radar/session/RirFeatureMeasurementTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "common/logging/AcceptanceText.h"
#include "common/radar/MtiMtdAcceptanceBank.h"
#include "common/radar/RadarEquations.h"
#include "remote_identification_radar/dwell/RirAntennaPatternRuntime.h"
#include "remote_identification_radar/runtime/PolarizationAcceptanceS.h"
#include "remote_identification_radar/runtime/RirAcceptanceLog.h"
#include "remote_identification_radar/tracking/RirTrackAssociator.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace runtime {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatPairDeg;
using oneq::logging::FormatSci;
using oneq::logging::FormatVec3;
using oneq::logging::YesNo;

constexpr double kLn10 = 2.302585092994046;
constexpr double kRadToDeg = 57.29577951308232;

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

double HeadingDeg(double ve, double vn) { return std::atan2(vn, ve) * kRadToDeg; }

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

void AppendDerivedRatio(std::string* text, const char* label, bool ok, double ratio_db,
                        const char* missing) {
  *text += label;
  if (ok) {
    *text += FormatF(ratio_db, 3);
    *text += "dB";
  } else {
    *text += missing;
  }
}

std::string DefaultAntennaCsvPath() {
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
  return std::string("rir_antenna_pattern.csv");
}

std::string DefaultScanCsvPath() {
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
  return std::string("rir_scan_pattern.csv");
}

}  // namespace

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
  const std::string id = "目标ID=" + std::to_string(input.target_id);
  const std::string range = " 斜距=" + FormatF(input.range_m, 1) + "m";
  const double echo_w = input.has_cell ? input.cell.echo_power_w : 0.0;
  (void)echo_w;
  const double thermal = input.has_cell ? input.cell.thermal_noise_power_w : 0.0;
  const double jam = input.has_cell ? input.cell.interference_power_w : 0.0;
  const double clutter = input.has_cell ? input.cell.clutter_power_w : 0.0;
  const double pc = input.has_cell ? input.cell.pulse_compression_gain : 1.0;
  const std::uint32_t pulses =
      input.has_cell ? input.cell.effective_pulse_count : 1U;
  const double pc_db = ToDb(pc);
  const double coherent_db = ToDb(static_cast<double>(std::max(1U, pulses)));
  const double target_db = static_cast<double>(input.gains.target_processing_gain_db);
  const double noise_db = static_cast<double>(input.gains.noise_processing_gain_db);
  const double clutter_db = static_cast<double>(input.gains.clutter_suppression_gain_db);
  const double jam_db = static_cast<double>(input.gains.jamming_suppression_gain_db);
  const double target_sum_db = pc_db + coherent_db + target_db;
  const double pc_noise = thermal * std::max(pc, 0.0);
  const double mti_residual = clutter_db <= 0.0 ? clutter : clutter / FromDb(clutter_db);
  oneq::common::radar::MtiMtdAcceptanceResult bank;
  const bool has_bank = TryBuildAcceptanceBank(input, &bank);
  const char* derived_missing =
      input.has_cell ? "暂无（PRF/载频非法，无法派生）" : "暂无（无 detection cell，无法派生）";

  std::string echo = id + range;
  echo += " SNR=" + FormatF(input.snr_db, 3) + "dB";
  echo += " 回波功率=" + FormatF(input.echo_power_dbw, 3) + "dBW";
  echo += " 热噪声=" + FormatSci(thermal) + "W";
  echo += " 干扰功率=" + FormatSci(jam) + "W";
  echo += " 杂波功率=" + FormatSci(clutter) + "W";
  RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.cycle, "回波功率计算", echo);

  std::string noise = id + " 热噪声功率=" + FormatSci(thermal) + "W SNR=" + FormatF(input.snr_db, 3) + "dB";
  RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.cycle, "接收机噪声功率", noise);

  std::string gain = id;
  gain += " 脉压=" + FormatF(pc_db, 3) + "dB";
  gain += " 相干积累=" + FormatF(coherent_db, 3) + "dB(N=" + std::to_string(pulses) + ")";
  gain += " 目标偏置=" + FormatF(target_db, 3) + "dB";
  gain += " 目标侧合计=" + FormatF(target_sum_db, 3) + "dB";
  gain += " 杂波抑制偏置=" + FormatF(clutter_db, 3) + "dB";
  if (has_bank) {
    gain += " 验收派生MTI增益=" + FormatF(bank.mti_gain_db, 3) + "dB";
    gain += " 验收派生MTD增益=" + FormatF(bank.mtd_gain_db, 3) + "dB（未进SINR）";
  } else {
    gain += " 验收派生MTI增益=";
    gain += derived_missing;
    gain += " 验收派生MTD增益=";
    gain += derived_missing;
  }
  RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.cycle, "目标信号增益", gain);

  std::string noise_gain = "热噪声=" + FormatSci(thermal) + "W";
  noise_gain += " 脉压后噪声=" + FormatSci(pc_noise) + "W";
  noise_gain += " 噪声处理偏置=" + FormatF(noise_db, 3) + "dB";
  if (has_bank) {
    noise_gain += " 验收派生多普勒通道噪声=" + FormatChannelWatts(bank.noise_w) + "W";
    noise_gain += " 验收派生MTD等效噪声=" + FormatSci(bank.mtd_equivalent_noise_w) + "W（未进SINR）";
  } else {
    noise_gain += " 验收派生多普勒通道噪声=";
    noise_gain += derived_missing;
    noise_gain += " 验收派生MTD等效噪声=";
    noise_gain += derived_missing;
  }
  noise_gain += " 噪声总增益=脉压线性增益+偏置";
  RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.cycle, "噪声增益", noise_gain);

  std::string clutter_text = id;
  clutter_text += " 主链偏置MTI后剩余杂波=" + FormatSci(mti_residual) + "W";
  clutter_text += " 杂波总抑制比=" + FormatF(clutter_db, 3) + "dB";
  if (has_bank) {
    clutter_text += " 验收派生MTI剩余杂波=" + FormatSci(bank.mti_residual_clutter_w) + "W";
    clutter_text += " 验收派生MTD通道分布=" + FormatChannelWatts(bank.clutter_w) + "W";
    double clutter_ratio_db = 0.0;
    const bool has_clutter_ratio = oneq::common::radar::TryAcceptancePowerRatioDb(
        clutter, SumChannelWatts(bank.clutter_w), &clutter_ratio_db);
    AppendDerivedRatio(&clutter_text, " 验收派生MTD杂波抑制比=", has_clutter_ratio, clutter_ratio_db,
                       "暂无（杂波输入或剩余非正，无法派生）");
    clutter_text += "（未进SINR）";
  } else {
    clutter_text += " 验收派生MTI剩余杂波=";
    clutter_text += derived_missing;
    clutter_text += " 验收派生MTD通道分布=";
    clutter_text += derived_missing;
    clutter_text += " 验收派生MTD杂波抑制比=";
    clutter_text += derived_missing;
  }
  RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.cycle, "杂波信号处理", clutter_text);

  std::string jam_text = id;
  jam_text += " 检测概率Pd=" + FormatF(input.pd, 5);
  jam_text += std::string(" 本周期检出=") + YesNo(input.detected);
  jam_text += range;
  jam_text += " 方位/俯仰=" + FormatPairDeg(input.look_az_deg, input.look_el_deg, 3) + "°";
  jam_text += " 干扰抑制偏置=" + FormatF(jam_db, 3) + "dB";
  if (has_bank && bank.has_jam_channels) {
    jam_text += " 验收派生MTI剩余干扰=" + FormatSci(bank.mti_residual_jam_w) + "W";
    jam_text += " 验收派生MTD通道干扰=" + FormatChannelWatts(bank.jam_w) + "W";
    const double jam_in = SumIncidentJamWatts(input);
    double mtd_jam_ratio_db = 0.0;
    double total_jam_ratio_db = 0.0;
    const bool has_mtd_jam = oneq::common::radar::TryAcceptancePowerRatioDb(
        jam_in, SumChannelWatts(bank.jam_w), &mtd_jam_ratio_db);
    const bool has_total_jam = oneq::common::radar::TryAcceptancePowerRatioDb(
        jam_in, bank.mti_residual_jam_w, &total_jam_ratio_db);
    AppendDerivedRatio(&jam_text, " 验收派生MTD干扰抑制比=", has_mtd_jam, mtd_jam_ratio_db,
                       "暂无（干扰输入或剩余非正，无法派生）");
    AppendDerivedRatio(&jam_text, " 验收派生总干扰抑制增益=", has_total_jam, total_jam_ratio_db,
                       "暂无（干扰输入或剩余非正，无法派生）");
    jam_text += "（未进SINR）";
  } else if (has_bank) {
    jam_text += " 验收派生MTI剩余干扰=无 验收派生MTD通道干扰=无";
    jam_text += " 验收派生MTD干扰抑制比=无 验收派生总干扰抑制增益=无";
  } else {
    jam_text += " 验收派生MTI剩余干扰=";
    jam_text += derived_missing;
    jam_text += " 验收派生MTD通道干扰=";
    jam_text += derived_missing;
    jam_text += " 验收派生MTD干扰抑制比=";
    jam_text += derived_missing;
    jam_text += " 验收派生总干扰抑制增益=";
    jam_text += derived_missing;
  }
  if (input.has_cell && std::isfinite(input.cfar_pfa) && input.cfar_pfa > 0.0 &&
      input.cfar_pfa < 1.0) {
    const double threshold = oneq::common::radar::RadarEquations::ComputeThreshold(
        input.cfar_pfa, static_cast<int>(std::max(1U, pulses)));
    if (std::isfinite(threshold)) {
      jam_text += " 验收派生统计检测门限=" + FormatF(threshold, 6) + "（未进判决）";
    } else {
      jam_text += " 验收派生统计检测门限=暂无（门限非有限）";
    }
  } else {
    jam_text += " 验收派生统计检测门限=";
    jam_text += input.has_cell ? "暂无（Pfa非法，无法派生）" : derived_missing;
  }
  RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.cycle, "干扰信号处理增益", jam_text);

  std::string rcs = id;
  rcs += " 本周期RCS=" + FormatF(input.rcs_m2, 3) + "m²";
  rcs += range;
  rcs += " 方位/俯仰=" + FormatPairDeg(input.look_az_deg, input.look_el_deg, 3) + "°";
  RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.cycle, "RCS实时探测", rcs);
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
                              float beam_el_deg, const std::string& found_targets) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::string content = "波束中心方位/俯仰=" + FormatPairDeg(beam_az_deg, beam_el_deg, 3) + "°";
  content += " 搜到目标=[" + found_targets + "]";
  content += " 指定角域裁剪后的搜索集合=无（实现仍遍历全部场景目标）";
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "指定空域搜索", content);
}

void WriteRirAssociation(float sim_time_sec, std::uint32_t cycle,
                         const tracking::RirAssociationResult& association) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::string content = "新量测=" + std::to_string(association.measurements.size());
  content += " 配到已有航迹=" + std::to_string(association.matches.size());
  content += " 未配上=" + std::to_string(association.missed_track_keys.size());
  if (!association.matches.empty()) {
    content += " 命中：";
    for (std::size_t i = 0; i < association.matches.size(); ++i) {
      if (i != 0U) {
        content += "；";
      }
      const tracking::RirAssociationMatch& match = association.matches[i];
      content += "航迹" + std::to_string(match.association_key) + "←量测" +
                 std::to_string(match.source_index) + " 代价=" + FormatF(match.cost, 4);
    }
  }
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "航迹关联", content);
}

void WriteRirTrackAndId(float sim_time_sec, std::uint32_t cycle, const tracking::RirTrackState& track,
                        const session::RirRecognitionResult* result,
                        const session::RirFeatureMeasurementRecord* features,
                        const std::vector<session::RirPolarizationRcsSample>* polarization_samples,
                        bool has_truth, double category_accuracy) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const double heading = HeadingDeg(track.velocity.x(), track.velocity.y());
  const double dt = 1.0;
  const std::string next_pos =
      FormatVec3(track.position.x() + track.velocity.x() * dt,
                 track.position.y() + track.velocity.y() * dt,
                 track.position.z() + track.velocity.z() * dt, 1);
  const int category = result != nullptr ? static_cast<int>(result->target_category) : -1;
  std::string measure = "目标ID=" + std::to_string(track.external_target_id);
  measure += " 斜距=无（滤波ENU） 方位/俯仰=无";
  measure += " 高度=" + FormatF(track.position.z(), 1) + "m";
  measure += " 速度=" + FormatF(track.speed, 3) + "m/s";
  measure += " 航向=" + FormatF(heading, 2) + "°";
  measure += " 加速度=" + FormatVec3(track.acceleration.x(), track.acceleration.y(),
                                     track.acceleration.z(), 4) +
             "m/s²";
  measure += " 飞机类型=识别大类枚举" + std::to_string(category);
  measure += " 舰船类型=无 车辆类型=无";
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "角度和距离测量", measure);

  std::string reentry = "航迹=" + std::to_string(track.association_key);
  reentry += " 目标ID=" + std::to_string(track.external_target_id);
  reentry += std::string(" 状态=") + TrackStatusText(track.status);
  reentry += " 高度=" + FormatF(track.position.z(), 1) + "m";
  reentry += " 速度=" + FormatF(track.speed, 3) + "m/s";
  reentry += " 航向=" + FormatF(heading, 2) + "°";
  reentry += " 加速度=" + FormatVec3(track.acceleration.x(), track.acceleration.y(),
                                     track.acceleration.z(), 4) +
             "m/s²";
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "典型/再入目标跟踪", reentry);

  const tracking::RirStateCovariance& cov = track.gaussian_state.covariance;
  std::string filter = "航迹=" + std::to_string(track.association_key);
  filter += " 当前ENU m=" + FormatVec3(track.position.x(), track.position.y(), track.position.z(), 1);
  filter += " 下一时刻预测ENU m=" + next_pos;
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
  filter += "] IMM权重=无";
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "跟踪滤波", filter);

  if (features != nullptr) {
    const auto& motion = features->features.motion;
    std::string motion_text = "航迹=" + std::to_string(track.association_key);
    motion_text += " 速度=" + FormatF(motion.speed_m_per_s, 3) + "m/s";
    motion_text += " 高度=" + FormatF(motion.altitude_m, 1) + "m";
    motion_text += std::string(" 近似直线=") + YesNo(motion.is_straight);
    motion_text += " 目标类别=大类枚举" + std::to_string(category);
    RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "运动特征处理", motion_text);

    const auto& rcs = features->features.rcs;
    std::string rcs_text = "航迹=" + std::to_string(track.association_key);
    rcs_text += " RCS均值/标准差=" + FormatF(rcs.mean_dbsm, 3) + "/" + FormatF(rcs.std_db, 3) + "dBsm";
    rcs_text += " 目标类别=大类枚举" + std::to_string(category);
    RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "RCS统计特征处理", rcs_text);

    const auto& pol = features->features.polarization;
    std::string pol_text = "航迹=" + std::to_string(track.association_key);
    pol_text += " 极化差/相对/和=" +
                FormatVec3(pol.energy_difference_db, pol.relative_difference_db, pol.energy_sum_db, 3);
    pol_text += " 目标类别=大类枚举" + std::to_string(category);
    pol_text += " 置信度=" + FormatF(result != nullptr ? result->confidence : 0.0, 4);
    pol_text += has_truth ? (" 正确识别率=" + FormatF(category_accuracy, 4))
                          : std::string(" 正确识别率=无（本场无真值对照）");
    PolarizationAcceptanceSResult derived;
    const bool has_s =
        polarization_samples != nullptr && features != nullptr &&
        TryResolvePolarizationAcceptanceS(*polarization_samples, features->look_az_deg,
                                          features->look_el_deg, &derived);
    if (has_s) {
      pol_text += " 验收派生功率迹=" + FormatSci(derived.span);
      pol_text += " 验收派生行列式=" + FormatSci(derived.abs_det);
      pol_text += " 验收派生去极化系数=" + FormatF(derived.depolarization, 4);
      pol_text += " 验收派生本征极化方向角=" + FormatF(derived.psi_deg, 3) + "°";
      pol_text += " 验收派生本征极化椭圆率=" + FormatF(derived.tau_deg, 3) + "（未进识别）";
    } else {
      const char* missing = "暂无（无交叉极化或HH-VV相位，无法按S派生）";
      pol_text += " 验收派生功率迹=";
      pol_text += missing;
      pol_text += " 验收派生行列式=";
      pol_text += missing;
      pol_text += " 验收派生去极化系数=";
      pol_text += missing;
      pol_text += " 验收派生本征极化方向角=";
      pol_text += missing;
      pol_text += " 验收派生本征极化椭圆率=";
      pol_text += missing;
    }
    RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "极化特征解算", pol_text);

    const auto& rp = features->features.range_profile;
    std::string rp_text = "航迹=" + std::to_string(track.association_key);
    rp_text += " 长度=" + FormatF(rp.length_m, 3) + "m";
    rp_text += " 峰数=" + std::to_string(rp.peak_count);
    rp_text += " 能量集中=" + FormatF(rp.peak_energy_concentration, 3);
    rp_text += " 分辨率=" + FormatF(rp.resolution_m, 3) + "m";
    rp_text += " 识别类型=大类枚举" + std::to_string(category);
    rp_text += " 置信度=" + FormatF(result != nullptr ? result->confidence : 0.0, 4);
    rp_text += " 散射中心列表=无";
    RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "宽带一维像特征解算", rp_text);
  }
}

void WriteRirSchedule(float sim_time_sec, std::uint32_t cycle, std::uint32_t planned,
                      std::uint32_t executed, float budget_sec, float consumed_sec,
                      std::uint32_t search_count, std::uint32_t track_count,
                      std::uint32_t ident_count) {
  if (!RIR_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::string content = "计划驻留/实际执行=" + std::to_string(planned) + "/" + std::to_string(executed);
  content += " 搜索=" + std::to_string(search_count);
  content += " 跟踪=" + std::to_string(track_count);
  content += " 识别=" + std::to_string(ident_count);
  content += " 验收派生事件执行列表=[搜索×" + std::to_string(search_count) + ",跟踪×" +
             std::to_string(track_count) + ",识别×" + std::to_string(ident_count) + "]";
  content += " 预算/已耗时=" + FormatF(budget_sec, 3) + "/" + FormatF(consumed_sec, 3) + "s";
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "调度策略", content);
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
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "初始化时间", "暂无（未做Session建链计时；门限≤100ms）");
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "单步执行时间", "暂无（生产Step未计时落盘；门限<20ms）");
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "单个模型加载时间", "暂无（识别库加载无时长记录）");
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "多模型并行加载", "暂无（库内无并行加载）");
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "典型场景和总仿真次数",
                      "场景=本会话 场景数=1 总仿真周期=结束时回写");
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
  const std::string csv_path = DefaultScanCsvPath();
  const bool wrote = TryExportRirScanPatternCsv(pattern, csv_path.c_str());
  std::string content = "波位总数=" + std::to_string(pattern.size());
  content += " 本周期驻留中心方位/俯仰=" + FormatPairDeg(az_deg, el_deg, 3) + "°";
  content += designate ? " 模式=指定" : " 模式=扫描";
  if (wrote && !pattern.empty()) {
    const std::uint64_t zero_based =
        cycle > 0U ? static_cast<std::uint64_t>(cycle - 1U) : 0U;
    const std::size_t index =
        static_cast<std::size_t>(zero_based % static_cast<std::uint64_t>(pattern.size()));
    const std::size_t next = (index + 1U) % pattern.size();
    content += " 验收派生波位排列表=" + csv_path;
    content += " 本周期序号=" + std::to_string(index);
    content += " 下一波位=" + FormatPairDeg(pattern[next].az_deg, pattern[next].el_deg, 3) + "°";
    if (designate) {
      content += "（本周期指向=指定，表为扫描序列，未进指向）";
    } else {
      content += "（未进指向）";
    }
  } else {
    content += " 验收派生波位排列表=暂无（扫描序列为空或未能写CSV）";
  }
  RIR_ACCEPTANCE_ITEM(sim_time_sec, cycle, "波束扫描", content);
}

bool TryExportRirAntennaPatternCsv(const config::hardware::RirAntennaConfig& antenna,
                                   const char* path) {
  const std::string out_path = (path != nullptr && *path != '\0') ? std::string(path)
                                                                  : DefaultAntennaCsvPath();
  std::ofstream out(out_path.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
  if (!out.is_open()) {
    return false;
  }
  out << "az_off_deg,el_off_deg,gain_dbi\n";
  dwell::RirAntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = antenna.nominal_az_beamwidth_deg;
  beamwidth.el_beamwidth_deg = antenna.nominal_el_beamwidth_deg;
  config::RirAzimuthElevationDeg pointing(0.0f, 0.0f);
  const float step = 2.0f;
  for (float az = -90.0f; az <= 90.0f + 0.5f * step; az += step) {
    for (float el = -90.0f; el <= 90.0f + 0.5f * step; el += step) {
      dwell::RirAntennaLookOffsetDeg offset;
      offset.delta_az_deg = az;
      offset.delta_el_deg = el;
      const dwell::RirAntennaPatternSample sample = dwell::RirEvaluateAntennaPattern(
          antenna.main_beam_gain_db, antenna.pattern, beamwidth, offset, pointing,
          antenna.antenna_length_m, antenna.antenna_width_m, 0.0f);
      out << FormatF(az, 1) << "," << FormatF(el, 1) << "," << FormatF(sample.gain_dbi, 3) << "\n";
    }
  }
  return true;
}

bool TryExportRirScanPatternCsv(
    const std::vector<oneq::common::radar::AzimuthElevationDeg>& pattern, const char* path) {
  const std::string out_path = (path != nullptr && *path != '\0') ? std::string(path)
                                                                  : DefaultScanCsvPath();
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
