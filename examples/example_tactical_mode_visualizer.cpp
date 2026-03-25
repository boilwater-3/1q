// Copyright 2026. All Rights Reserved.
//
// @file example_tactical_mode_visualizer.cpp
// @brief ImGui + ImPlot 战术模式切换场景可视化 Demo。
//
// 仿真场景：
//   目标 A (ID=5001)：(10km, 1km, 5km)，速度 (-200, -20, 0) m/s，RCS 2.0 m²
//   目标 B (ID=5002)：(8km, -1km, 4km)，速度 (-180, 30, -5) m/s，RCS 1.5 m²
//
//   共 35 个 cycle，dt=0.5s，环境威胁逐步升级：
//
//   Phase 1 (cycle 0~7):   无干扰 → Baseline 模式
//   Phase 2 (cycle 8~14):  旁瓣噪声干扰 → 触发 ThreatResponse
//   Phase 3 (cycle 15~21): 主瓣欺骗+噪声复合干扰 → 强化 ECCM
//   Phase 4 (cycle 22~28): 全谱高强度干扰 → ProtectedEmission
//   Phase 5 (cycle 29~34): 干扰撤销 → 恢复/降级观察
//
// 可视化面板：
//   左上 — XY 轨迹俯视图
//   左下 — 指令 Profile 时序（LPI/ECCM 各标志随 cycle 变化）
//   右上 — 轨迹状态 + 指令摘要表
//   右下 — 当前战术态势面板（模式、指令计数、Profile 状态）

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "1q/airborne_radar/common/ConfigPresets.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/common/TargetFeatureUtils.h"
#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/session/RadarCycleResult.h"
#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"

namespace {

// ── 仿真参数 ──────────────────────────────────────────────────────────────────

constexpr int kMaxCycles = 35;
constexpr float kDtSec = 0.5f;

// 阶段划分
constexpr int kPhase1End = 8;
constexpr int kPhase2End = 15;
constexpr int kPhase3End = 22;
constexpr int kPhase4End = 29;

struct TargetInit {
  std::uint64_t id;
  float px, py, pz;
  float vx, vy, vz;
  float rcs;
};

constexpr TargetInit kTargets[] = {
    {5001, 10000.0f, 1000.0f, 5000.0f, -200.0f, -20.0f, 0.0f, 2.0f},
    {5002, 8000.0f, -1000.0f, 4000.0f, -180.0f, 30.0f, -5.0f, 1.5f},
};

// ── 阶段信息 ────────────────────────────────────────────────────────────────

struct PhaseInfo {
  const char* name;
  ImVec4 color;
};

PhaseInfo GetPhase(int cycle) {
  if (cycle < kPhase1End) return {"Baseline (Clean)", {0.4f, 1.0f, 0.4f, 1.0f}};
  if (cycle < kPhase2End) return {"ThreatResponse (Noise)", {1.0f, 0.8f, 0.2f, 1.0f}};
  if (cycle < kPhase3End) return {"ECCM Escalation (N+D)", {1.0f, 0.5f, 0.2f, 1.0f}};
  if (cycle < kPhase4End) return {"ProtectedEmission (Full)", {1.0f, 0.2f, 0.2f, 1.0f}};
  return {"Recovery", {0.4f, 0.8f, 1.0f, 1.0f}};
}

// ── 颜色辅助 ──────────────────────────────────────────────────────────────────

ImVec4 TrackColor(std::uint64_t association_key) {
  static const ImVec4 kPalette[] = {
      {0.12f, 0.47f, 0.71f, 1.0f}, {1.00f, 0.50f, 0.05f, 1.0f}, {0.17f, 0.63f, 0.17f, 1.0f},
      {0.84f, 0.15f, 0.16f, 1.0f}, {0.58f, 0.40f, 0.74f, 1.0f}, {0.55f, 0.34f, 0.29f, 1.0f},
      {0.89f, 0.47f, 0.76f, 1.0f},
  };
  return kPalette[association_key % 7];
}

// ── 仿真状态 ──────────────────────────────────────────────────────────────────

struct SimState {
  struct TargetPos {
    float px, py, pz;
    float vx, vy, vz;
    float rcs;
  };
  std::map<std::uint64_t, TargetPos> target_pos;
  int current_cycle{0};
  bool finished{false};

  // 轨迹历史
  std::map<std::uint64_t, std::vector<float>> hist_x;
  std::map<std::uint64_t, std::vector<float>> hist_y;

  // 最近一帧
  std::vector<airborne_radar::common::DecisionTrackSnapshot> latest_tracks;

  // 指令历史
  std::vector<std::vector<airborne_radar::common::RadarCommand>> command_history;

  // Profile 历史（逐 cycle 记录各标志）
  std::vector<bool> hist_lpi_power;
  std::vector<bool> hist_lpi_beam;
  std::vector<bool> hist_slc;
  std::vector<bool> hist_abf;
  std::vector<bool> hist_freq_agility;
  std::vector<bool> hist_rejitter;
  std::vector<float> hist_burnthrough;

  // 干扰强度历史
  std::vector<float> hist_jam_severity;

  // 累计指令计数
  std::map<airborne_radar::common::RadarCommandType, int> total_cmd_count;

  void Reset() {
    current_cycle = 0;
    finished = false;
    hist_x.clear();
    hist_y.clear();
    latest_tracks.clear();
    command_history.clear();
    hist_lpi_power.clear();
    hist_lpi_beam.clear();
    hist_slc.clear();
    hist_abf.clear();
    hist_freq_agility.clear();
    hist_rejitter.clear();
    hist_burnthrough.clear();
    hist_jam_severity.clear();
    total_cmd_count.clear();
    target_pos.clear();
    for (const auto& t : kTargets) {
      target_pos[t.id] = {t.px, t.py, t.pz, t.vx, t.vy, t.vz, t.rcs};
    }
  }
};

// ── 构造 RadarSession ─────────────────────────────────────────────────────────

std::unique_ptr<airborne_radar::core::session::RadarSession> MakeSession() {
  namespace aq = airborne_radar::common;
  auto session = std::unique_ptr<airborne_radar::core::session::RadarSession>(
      new airborne_radar::core::session::RadarSession(
          aq::RadarSessionConfigBuilder(aq::MakeDetectionMissionRadarSessionConfig())
              .EnablePhysicsDetection()
              .WithTransmitterPeakPowerW(5e6f)
              .WithTransmitterFrequencyHz(9.3e9f)
              .WithTransmitterBandwidthHz(10e6f)
              .WithTransmitterPulseWidthS(20e-6f)
              .WithTransmitterPrfHz(500.0f)
              .WithAntennaMainBeamGainDb(38.0f)
              .WithAntennaNominalBeamwidthDeg(3.5f, 3.5f)
              .WithReceiverNoiseFigureDb(3.5f)
              .WithJammingDetectionThresholdDb(4.0f)
              .Build()));

  airborne_radar::environment::EnvironmentModelConfig env_cfg;
  env_cfg.clutter_power_db = -200.0f;
  env_cfg.base_propagation_loss_db = 0.0f;
  env_cfg.atmospheric_attenuation_db = 0.0f;
  env_cfg.terrain_reflection_db = 0.0f;
  session->UpdateEnvironmentModelConfig(env_cfg);

  return session;
}

// ── 构造当前 cycle 的环境场景 ─────────────────────────────────────────────────

airborne_radar::environment::EnvironmentSceneState BuildScene(int cycle) {
  using airborne_radar::environment::EnvironmentSceneBuilder;
  using airborne_radar::environment::JammerEmitterState;
  using airborne_radar::environment::JammingTechnique;

  EnvironmentSceneBuilder builder;
  builder.SetClutterPowerDb(-200.0f)
      .SetBasePropagationLossDb(0.0f)
      .SetAtmosphericAttenuationDb(0.0f)
      .SetTerrainReflectionDb(0.0f);

  // Phase 2: 旁瓣噪声
  if (cycle >= kPhase1End && cycle < kPhase4End) {
    JammerEmitterState noise;
    noise.technique = JammingTechnique::kNoiseSuppression;
    noise.power_db = 12.0f + static_cast<float>(std::min(cycle - kPhase1End, 10)) * 0.5f;
    noise.js_db = 8.0f;
    noise.frequency_overlap_ratio = 0.30f;
    noise.prf_lock_risk = 0.15f;
    noise.azimuth_deg = 20.0f;
    noise.in_sidelobe = true;
    noise.confidence = 0.9f;
    builder.AddNoiseJammer(noise);
  }

  // Phase 3: 主瓣欺骗
  if (cycle >= kPhase2End && cycle < kPhase4End) {
    JammerEmitterState deception;
    deception.technique = JammingTechnique::kDeception;
    deception.power_db = 10.0f + static_cast<float>(std::min(cycle - kPhase2End, 7)) * 0.8f;
    deception.js_db = 7.0f;
    deception.frequency_overlap_ratio = 0.85f;
    deception.prf_lock_risk = 0.50f;
    deception.azimuth_deg = -2.0f;
    deception.in_sidelobe = false;
    deception.confidence = 0.85f;
    builder.AddDeceptionJammer(deception);
  }

  // Phase 4: 全谱高强度（叠加转发）
  if (cycle >= kPhase3End && cycle < kPhase4End) {
    JammerEmitterState repeater;
    repeater.technique = JammingTechnique::kRepeater;
    repeater.power_db = 15.0f;
    repeater.js_db = 12.0f;
    repeater.frequency_overlap_ratio = 0.95f;
    repeater.prf_lock_risk = 0.70f;
    repeater.azimuth_deg = 0.0f;
    repeater.in_sidelobe = false;
    repeater.confidence = 0.80f;
    builder.AddRepeaterJammer(repeater);
  }

  return builder.Build();
}

// ── 推进一个 cycle ────────────────────────────────────────────────────────────

void StepOnce(airborne_radar::core::session::RadarSession& session, SimState& sim) {
  if (sim.finished || sim.current_cycle >= kMaxCycles) {
    sim.finished = true;
    return;
  }

  namespace aq = airborne_radar::common;
  using airborne_radar::core::context::RadarCycleInput;

  RadarCycleInput input;
  input.dt_sec = kDtSec;
  for (const auto& kv : sim.target_pos) {
    const auto& p = kv.second;
    input.target_features.push_back(
        aq::MakeAirTarget(kv.first, p.px, p.py, p.pz, p.vx, p.vy, p.vz, p.rcs));
  }

  airborne_radar::core::session::RadarCycleResult result;
  if (sim.current_cycle >= kPhase1End && sim.current_cycle < kPhase4End) {
    auto scene = BuildScene(sim.current_cycle);
    result = session.StepWithResult(input, scene);
  } else {
    result = session.StepWithResult(input);
  }

  // 轨迹历史
  for (const auto& snap : result.track_output_frame.tracks) {
    const auto key = snap.state.association_key;
    sim.hist_x[key].push_back(snap.state.position_x / 1000.0f);
    sim.hist_y[key].push_back(snap.state.position_y / 1000.0f);
  }
  sim.latest_tracks = result.track_output_frame.tracks;

  // 指令历史
  sim.command_history.push_back(result.submitted_commands);
  for (const auto& cmd : result.submitted_commands) {
    sim.total_cmd_count[cmd.type]++;
  }

  // Profile 历史
  if (result.has_control_profile) {
    const auto& p = result.control_profile;
    sim.hist_lpi_power.push_back(p.enable_lpi_power_control);
    sim.hist_lpi_beam.push_back(p.enable_lpi_beamforming);
    sim.hist_slc.push_back(p.enable_sidelobe_canceller);
    sim.hist_abf.push_back(p.enable_adaptive_beamforming);
    sim.hist_freq_agility.push_back(p.enable_agility_frequency);
    sim.hist_rejitter.push_back(p.enable_eccm_rejitter);
    sim.hist_burnthrough.push_back(p.eccm_burnthrough_gain);
  } else {
    sim.hist_lpi_power.push_back(false);
    sim.hist_lpi_beam.push_back(false);
    sim.hist_slc.push_back(false);
    sim.hist_abf.push_back(false);
    sim.hist_freq_agility.push_back(false);
    sim.hist_rejitter.push_back(false);
    sim.hist_burnthrough.push_back(1.0f);
  }

  sim.hist_jam_severity.push_back(result.association_quality_metrics.jamming_severity);

  // 推进目标位置
  for (auto& kv : sim.target_pos) {
    kv.second.px += kv.second.vx * kDtSec;
    kv.second.py += kv.second.vy * kDtSec;
    kv.second.pz += kv.second.vz * kDtSec;
  }

  sim.current_cycle++;
  if (sim.current_cycle >= kMaxCycles) {
    sim.finished = true;
  }
}

// ── 状态字符串 ────────────────────────────────────────────────────────────────

const char* StatusStr(airborne_radar::common::DecisionTrackStatus s) {
  switch (s) {
    case airborne_radar::common::DecisionTrackStatus::kTentative:
      return "Tentative";
    case airborne_radar::common::DecisionTrackStatus::kConfirmed:
      return "Confirmed";
    case airborne_radar::common::DecisionTrackStatus::kLost:
      return "Lost";
  }
  return "Unknown";
}

ImPlotMarker StatusMarker(airborne_radar::common::DecisionTrackStatus s) {
  switch (s) {
    case airborne_radar::common::DecisionTrackStatus::kTentative:
      return ImPlotMarker_Circle;
    case airborne_radar::common::DecisionTrackStatus::kConfirmed:
      return ImPlotMarker_Square;
    case airborne_radar::common::DecisionTrackStatus::kLost:
      return ImPlotMarker_Cross;
  }
  return ImPlotMarker_Circle;
}

const char* CmdTypeName(airborne_radar::common::RadarCommandType t) {
  using T = airborne_radar::common::RadarCommandType;
  switch (t) {
    case T::NONE:
      return "NONE";
    case T::SET_LPI_POWER:
      return "LPI_Power";
    case T::SET_LPI_BEAMFORMING:
      return "LPI_Beam";
    case T::SET_LPI_DWELL:
      return "LPI_Dwell";
    case T::ENABLE_SIDELOBE_CANCELLER:
      return "SLC";
    case T::ENABLE_ADAPTIVE_BEAMFORMING:
      return "ABF";
    case T::SET_AGILITY_FREQ:
      return "FreqAgility";
    case T::SET_ECCM_REJITTER:
      return "Rejitter";
    case T::SET_ECCM_BURNTHROUGH_GAIN:
      return "Burnthrough";
  }
  return "?";
}

// ── 渲染 XY 俯视图 ──────────────────────────────────────────────────────────

void RenderXYPlot(const SimState& sim) {
  if (!ImPlot::BeginPlot("XY Top-Down View", ImVec2(-1, -1), ImPlotFlags_Equal)) {
    return;
  }
  ImPlot::SetupAxes("X (km)", "Y (km)");

  for (const auto& kv : sim.hist_x) {
    const auto key = kv.first;
    const auto& xs = kv.second;
    const auto& ys = sim.hist_y.at(key);
    if (xs.empty()) continue;

    ImVec4 col = TrackColor(key);
    ImPlot::SetNextLineStyle({col.x, col.y, col.z, col.w}, 1.5f);
    char label[32];
    std::snprintf(label, sizeof(label), "Track %llu", static_cast<unsigned long long>(key));
    ImPlot::PlotLine(label, xs.data(), ys.data(), static_cast<int>(xs.size()));

    ImPlotMarker marker = ImPlotMarker_Circle;
    for (const auto& snap : sim.latest_tracks) {
      if (snap.state.association_key == key) {
        marker = StatusMarker(snap.state.status);
        break;
      }
    }
    ImPlot::SetNextMarkerStyle(marker, 8.0f, {col.x, col.y, col.z, col.w});
    char slabel[48];
    std::snprintf(slabel, sizeof(slabel), "Pos %llu", static_cast<unsigned long long>(key));
    ImPlot::PlotScatter(slabel, &xs.back(), &ys.back(), 1);
  }

  ImPlot::EndPlot();
}

// ── 渲染 Profile 时序图 ────────────────────────────────────────────────────

void RenderProfileTimeline(const SimState& sim) {
  if (!ImPlot::BeginPlot("Control Profile Timeline", ImVec2(-1, -1))) {
    return;
  }
  ImPlot::SetupAxes("Cycle", "");
  ImPlot::SetupAxisLimits(ImAxis_X1, 0, kMaxCycles);
  ImPlot::SetupAxisLimits(ImAxis_Y1, -0.5, 8.5);

  // 每个 Profile 标志占一行
  struct FlagRow {
    const std::vector<bool>* data;
    float y;
    ImVec4 color;
    const char* name;
  };
  const FlagRow rows[] = {
      {&sim.hist_lpi_power, 0.0f, {0.6f, 0.4f, 0.8f, 1.0f}, "LPI Power"},
      {&sim.hist_lpi_beam, 1.0f, {0.5f, 0.3f, 0.7f, 1.0f}, "LPI Beam"},
      {&sim.hist_slc, 2.0f, {0.2f, 0.8f, 0.2f, 1.0f}, "SLC"},
      {&sim.hist_abf, 3.0f, {0.2f, 0.6f, 1.0f, 1.0f}, "ABF"},
      {&sim.hist_freq_agility, 4.0f, {1.0f, 0.8f, 0.2f, 1.0f}, "FreqAgility"},
      {&sim.hist_rejitter, 5.0f, {1.0f, 0.5f, 0.2f, 1.0f}, "Rejitter"},
  };

  for (const auto& row : rows) {
    if (row.data->empty()) continue;
    std::vector<float> xs;
    std::vector<float> ys;
    for (std::size_t i = 0; i < row.data->size(); ++i) {
      if ((*row.data)[i]) {
        xs.push_back(static_cast<float>(i));
        ys.push_back(row.y);
      }
    }
    if (!xs.empty()) {
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Square, 5.0f,
                                 {row.color.x, row.color.y, row.color.z, row.color.w});
      ImPlot::PlotScatter(row.name, xs.data(), ys.data(), static_cast<int>(xs.size()));
    }
  }

  // Burnthrough 增益曲线
  if (!sim.hist_burnthrough.empty()) {
    std::vector<float> cycles(sim.hist_burnthrough.size());
    std::vector<float> scaled(sim.hist_burnthrough.size());
    for (std::size_t i = 0; i < cycles.size(); ++i) {
      cycles[i] = static_cast<float>(i);
      scaled[i] = 6.0f + (sim.hist_burnthrough[i] - 1.0f) * 2.0f;
    }
    ImPlot::SetNextLineStyle({1.0f, 0.2f, 0.2f, 0.8f}, 1.5f);
    ImPlot::PlotLine("Burnthrough", cycles.data(), scaled.data(), static_cast<int>(cycles.size()));
  }

  // 干扰强度曲线
  if (!sim.hist_jam_severity.empty()) {
    std::vector<float> cycles(sim.hist_jam_severity.size());
    std::vector<float> scaled(sim.hist_jam_severity.size());
    for (std::size_t i = 0; i < cycles.size(); ++i) {
      cycles[i] = static_cast<float>(i);
      scaled[i] = 7.0f + sim.hist_jam_severity[i] * 1.5f;
    }
    ImPlot::SetNextLineStyle({1.0f, 0.3f, 0.3f, 0.5f}, 1.0f);
    ImPlot::PlotLine("JamSeverity", cycles.data(), scaled.data(), static_cast<int>(cycles.size()));
  }

  ImPlot::EndPlot();
}

// ── 渲染轨迹 + 指令摘要表 ──────────────────────────────────────────────────

void RenderTrackAndCmdTable(const SimState& sim) {
  auto phase = GetPhase(sim.current_cycle);
  ImGui::TextColored(phase.color, "Mode: %s", phase.name);
  ImGui::Text("Cycle %d / %d", sim.current_cycle, kMaxCycles);
  ImGui::Separator();

  // 轨迹表
  if (ImGui::BeginTable(
          "tracks", 6,
          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
    ImGui::TableSetupColumn("ID");
    ImGui::TableSetupColumn("Status");
    ImGui::TableSetupColumn("Speed");
    ImGui::TableSetupColumn("Hit");
    ImGui::TableSetupColumn("Miss");
    ImGui::TableSetupColumn("Jam");
    ImGui::TableHeadersRow();

    for (const auto& snap : sim.latest_tracks) {
      const auto& st = snap.state;
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextColored(TrackColor(st.association_key), "%llu",
                         static_cast<unsigned long long>(st.association_key));
      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(StatusStr(st.status));
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.0f", static_cast<double>(st.speed));
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%u", st.hit_count);
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%u", st.miss_count);
      ImGui::TableSetColumnIndex(5);
      if (st.jamming_detected) {
        ImGui::TextColored({1.0f, 0.3f, 0.3f, 1.0f}, "!");
      } else {
        ImGui::TextUnformatted("-");
      }
    }
    ImGui::EndTable();
  }

  // 本 cycle 指令摘要
  ImGui::Spacing();
  ImGui::Text("Commands this cycle:");
  if (!sim.command_history.empty()) {
    const auto& cmds = sim.command_history.back();
    if (cmds.empty()) {
      ImGui::TextDisabled("  (none)");
    } else {
      for (const auto& cmd : cmds) {
        ImGui::BulletText("%s", CmdTypeName(cmd.type));
      }
    }
  }
}

// ── 渲染战术态势面板 ────────────────────────────────────────────────────────

void RenderTacticalPanel(const SimState& sim) {
  ImGui::Text("Tactical Situation");
  ImGui::Separator();

  auto phase = GetPhase(sim.current_cycle);
  ImGui::TextColored(phase.color, "Current Mode: %s", phase.name);

  ImGui::Spacing();
  ImGui::Text("Cumulative Command Count:");
  for (const auto& kv : sim.total_cmd_count) {
    if (kv.first == airborne_radar::common::RadarCommandType::NONE) continue;
    ImGui::Text("  %-15s : %d", CmdTypeName(kv.first), kv.second);
  }

  ImGui::Spacing();
  if (!sim.hist_jam_severity.empty()) {
    float sev = sim.hist_jam_severity.back();
    ImVec4 sev_col = (sev < 0.2f)   ? ImVec4{0.4f, 1.0f, 0.4f, 1.0f}
                     : (sev < 0.5f) ? ImVec4{1.0f, 0.8f, 0.2f, 1.0f}
                                    : ImVec4{1.0f, 0.3f, 0.3f, 1.0f};
    ImGui::TextColored(sev_col, "Jamming Severity: %.3f", static_cast<double>(sev));
  }

  // Profile 概览
  ImGui::Spacing();
  ImGui::Text("Active Capabilities:");
  auto BoolTag = [](const char* label, bool active) {
    if (active) {
      ImGui::SameLine();
      ImGui::TextColored({0.2f, 1.0f, 0.2f, 1.0f}, "[%s]", label);
    }
  };
  ImGui::TextUnformatted(" ");
  if (!sim.hist_lpi_power.empty()) {
    BoolTag("LPI", sim.hist_lpi_power.back());
    BoolTag("SLC", sim.hist_slc.back());
    BoolTag("ABF", sim.hist_abf.back());
    BoolTag("FA", sim.hist_freq_agility.back());
    BoolTag("RJ", sim.hist_rejitter.back());
  }
}

}  // namespace

// ── main ──────────────────────────────────────────────────────────────────────

int main() {
  if (!glfwInit()) {
    std::fprintf(stderr, "GLFW init failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  GLFWwindow* window =
      glfwCreateWindow(1400, 900, "Airborne Radar - Tactical Mode Transition", nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "GLFW window creation failed\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  SimState sim;
  sim.Reset();
  auto session = MakeSession();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    int display_w = 0, display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({static_cast<float>(display_w), static_cast<float>(display_h)});
    ImGui::Begin("##main", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

    // 顶部控制栏
    if (ImGui::Button("Step") && !sim.finished) {
      StepOnce(*session, sim);
    }
    ImGui::SameLine();
    if (ImGui::Button("Run All")) {
      while (!sim.finished) {
        StepOnce(*session, sim);
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
      session = MakeSession();
      sim.Reset();
    }
    ImGui::SameLine();
    auto ph = GetPhase(sim.current_cycle);
    ImGui::TextColored(ph.color, "[%s]", ph.name);
    if (sim.finished) {
      ImGui::SameLine();
      ImGui::TextColored({0.4f, 1.0f, 0.4f, 1.0f}, "Complete.");
    }

    ImGui::Separator();

    float total_w = ImGui::GetContentRegionAvail().x;
    float total_h = ImGui::GetContentRegionAvail().y;
    float left_w = total_w * 0.6f;

    // 左侧
    ImGui::BeginChild("left", {left_w, 0.0f}, false);
    {
      ImGui::BeginChild("xy", {0.0f, total_h * 0.5f}, false);
      RenderXYPlot(sim);
      ImGui::EndChild();

      ImGui::BeginChild("profile_tl", {0.0f, 0.0f}, false);
      RenderProfileTimeline(sim);
      ImGui::EndChild();
    }
    ImGui::EndChild();

    ImGui::SameLine();

    // 右侧
    ImGui::BeginChild("right", {0.0f, 0.0f}, false);
    {
      ImGui::BeginChild("track_cmd", {0.0f, total_h * 0.55f}, true);
      RenderTrackAndCmdTable(sim);
      ImGui::EndChild();

      ImGui::BeginChild("tactical", {0.0f, 0.0f}, true);
      RenderTacticalPanel(sim);
      ImGui::EndChild();
    }
    ImGui::EndChild();

    ImGui::End();

    ImGui::Render();
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
