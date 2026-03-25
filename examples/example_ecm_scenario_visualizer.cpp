// Copyright 2026. All Rights Reserved.
//
// @file example_ecm_scenario_visualizer.cpp
// @brief ImGui + ImPlot 电子对抗场景可视化 Demo。
//
// 仿真场景：
//   目标 A (ID=2001)：正前方逼近 (-250, 20, -5) m/s，RCS 2.0 m²
//   目标 B (ID=2002)：侧向逼近 (-180, -60, 0) m/s，RCS 1.5 m²
//   目标 C (ID=2003)：远距高速 (-350, 10, -8) m/s，RCS 3.0 m²
//
//   共 30 个 cycle，dt=0.5s：
//     cycle  0~9 : 无干扰（正常跟踪基线）
//     cycle 10~14: 注入噪声压制干扰（旁瓣）
//     cycle 15~19: 叠加欺骗干扰（主瓣）
//     cycle 20~24: 叠加转发干扰（全向）
//     cycle 25~29: 撤销所有干扰（观察恢复过程）
//
// 可视化面板：
//   左上 — 轨迹 XY 俯视图
//   左下 — ECCM 指令时间线（显示每个 cycle 下发的指令类型）
//   右上 — 轨迹状态表格
//   右下 — 控制 Profile 状态面板（LPI/ECCM 标志实时展示）

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

namespace {

// ── 仿真参数 ──────────────────────────────────────────────────────────────────

constexpr int kMaxCycles = 30;
constexpr float kDtSec = 0.5f;

// 干扰阶段划分（cycle 0~9 为无干扰基线）
constexpr int kPhaseNoise = 10;      // cycle 10~14
constexpr int kPhaseDeception = 15;  // cycle 15~19
constexpr int kPhaseRepeater = 20;   // cycle 20~24
constexpr int kPhaseRecovery = 25;   // cycle 25~29

struct TargetInit {
  std::uint64_t id;
  float px, py, pz;
  float vx, vy, vz;
  float rcs;
};

constexpr TargetInit kTargets[] = {
    {2001, 8000.0f, 500.0f, 400.0f, -250.0f, 20.0f, -5.0f, 2.0f},
    {2002, 6000.0f, -2000.0f, 600.0f, -180.0f, -60.0f, 0.0f, 1.5f},
    {2003, 12000.0f, 300.0f, 700.0f, -350.0f, 10.0f, -8.0f, 3.0f},
};

// ── 颜色辅助 ──────────────────────────────────────────────────────────────────

ImVec4 TrackColor(std::uint64_t association_key) {
  static const ImVec4 kPalette[] = {
      {0.12f, 0.47f, 0.71f, 1.0f}, {1.00f, 0.50f, 0.05f, 1.0f}, {0.17f, 0.63f, 0.17f, 1.0f},
      {0.84f, 0.15f, 0.16f, 1.0f}, {0.58f, 0.40f, 0.74f, 1.0f}, {0.55f, 0.34f, 0.29f, 1.0f},
      {0.89f, 0.47f, 0.76f, 1.0f},
  };
  return kPalette[association_key % 7];
}

// ── 指令名称映射 ────────────────────────────────────────────────────────────

const char* CommandTypeName(airborne_radar::common::RadarCommandType t) {
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

const char* PhaseName(int cycle) {
  if (cycle < kPhaseNoise) return "Clean";
  if (cycle < kPhaseDeception) return "Noise";
  if (cycle < kPhaseRepeater) return "Noise+Deception";
  if (cycle < kPhaseRecovery) return "Noise+Deception+Repeater";
  return "Recovery";
}

ImVec4 PhaseColor(int cycle) {
  if (cycle < kPhaseNoise) return {0.4f, 1.0f, 0.4f, 1.0f};
  if (cycle < kPhaseDeception) return {1.0f, 0.8f, 0.2f, 1.0f};
  if (cycle < kPhaseRepeater) return {1.0f, 0.5f, 0.2f, 1.0f};
  if (cycle < kPhaseRecovery) return {1.0f, 0.2f, 0.2f, 1.0f};
  return {0.4f, 0.8f, 1.0f, 1.0f};
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

  // 轨迹历史
  std::map<std::uint64_t, std::vector<float>> hist_x;
  std::map<std::uint64_t, std::vector<float>> hist_y;

  // 最近一帧轨迹
  std::vector<airborne_radar::common::DecisionTrackSnapshot> latest_tracks;

  // 指令历史（每个 cycle 的指令列表）
  std::vector<std::vector<airborne_radar::common::RadarCommand>> command_history;

  // 控制 Profile 历史
  std::vector<airborne_radar::common::RadarControlProfile> profile_history;

  // 关联质量历史
  std::vector<float> jamming_severity_history;

  bool finished{false};

  void Reset() {
    current_cycle = 0;
    hist_x.clear();
    hist_y.clear();
    latest_tracks.clear();
    command_history.clear();
    profile_history.clear();
    jamming_severity_history.clear();
    finished = false;
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
              .WithJammingDetectionThresholdDb(5.0f)
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

  // 噪声压制干扰（cycle 10~24）
  if (cycle >= kPhaseNoise && cycle < kPhaseRecovery) {
    JammerEmitterState noise_jammer;
    noise_jammer.technique = JammingTechnique::kNoiseSuppression;
    noise_jammer.power_db = 14.0f;
    noise_jammer.js_db = 10.0f;
    noise_jammer.frequency_overlap_ratio = 0.30f;
    noise_jammer.prf_lock_risk = 0.15f;
    noise_jammer.azimuth_deg = 15.0f;
    noise_jammer.in_sidelobe = true;
    noise_jammer.confidence = 0.9f;
    builder.AddNoiseJammer(noise_jammer);
  }

  // 欺骗干扰（cycle 15~24）
  if (cycle >= kPhaseDeception && cycle < kPhaseRecovery) {
    JammerEmitterState deception_jammer;
    deception_jammer.technique = JammingTechnique::kDeception;
    deception_jammer.power_db = 10.0f;
    deception_jammer.js_db = 6.0f;
    deception_jammer.frequency_overlap_ratio = 0.80f;
    deception_jammer.prf_lock_risk = 0.40f;
    deception_jammer.azimuth_deg = -5.0f;
    deception_jammer.in_sidelobe = false;
    deception_jammer.confidence = 0.85f;
    builder.AddDeceptionJammer(deception_jammer);
  }

  // 转发干扰（cycle 20~24）
  if (cycle >= kPhaseRepeater && cycle < kPhaseRecovery) {
    JammerEmitterState repeater_jammer;
    repeater_jammer.technique = JammingTechnique::kRepeater;
    repeater_jammer.power_db = 8.0f;
    repeater_jammer.js_db = 5.0f;
    repeater_jammer.frequency_overlap_ratio = 0.95f;
    repeater_jammer.prf_lock_risk = 0.60f;
    repeater_jammer.azimuth_deg = 3.0f;
    repeater_jammer.in_sidelobe = false;
    repeater_jammer.confidence = 0.75f;
    builder.AddRepeaterJammer(repeater_jammer);
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

  // 根据当前 cycle 构造环境场景
  airborne_radar::core::session::RadarCycleResult result;
  if (sim.current_cycle >= kPhaseNoise && sim.current_cycle < kPhaseRecovery) {
    auto scene = BuildScene(sim.current_cycle);
    result = session.StepWithResult(input, scene);
  } else {
    result = session.StepWithResult(input);
  }

  // 追加轨迹历史
  for (const auto& snap : result.track_output_frame.tracks) {
    const auto key = snap.state.association_key;
    sim.hist_x[key].push_back(snap.state.position_x / 1000.0f);
    sim.hist_y[key].push_back(snap.state.position_y / 1000.0f);
  }
  sim.latest_tracks = result.track_output_frame.tracks;

  // 记录指令历史
  sim.command_history.push_back(result.submitted_commands);

  // 记录控制 Profile
  if (result.has_control_profile) {
    sim.profile_history.push_back(result.control_profile);
  } else {
    sim.profile_history.push_back({});
  }

  // 记录干扰强度
  sim.jamming_severity_history.push_back(result.association_quality_metrics.jamming_severity);

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

// ── 渲染轨迹 XY 俯视图 ──────────────────────────────────────────────────────

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

// ── 渲染轨迹状态表格 ──────────────────────────────────────────────────────────

void RenderTrackTable(const SimState& sim) {
  ImGui::TextColored(PhaseColor(sim.current_cycle), "Phase: %s", PhaseName(sim.current_cycle));
  ImGui::Text("Cycle %d / %d", sim.current_cycle, kMaxCycles);
  ImGui::Separator();

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
}

// ── 渲染 ECCM 指令时间线 ────────────────────────────────────────────────────

void RenderCommandTimeline(const SimState& sim) {
  if (!ImPlot::BeginPlot("ECCM Command Timeline", ImVec2(-1, -1))) {
    return;
  }
  ImPlot::SetupAxes("Cycle", "");
  ImPlot::SetupAxisLimits(ImAxis_X1, 0, kMaxCycles);

  // 为每种指令类型绘制散点标记
  using CmdType = airborne_radar::common::RadarCommandType;
  struct CmdRow {
    CmdType type;
    float y_pos;
    ImVec4 color;
  };
  const CmdRow rows[] = {
      {CmdType::ENABLE_SIDELOBE_CANCELLER, 1.0f, {0.2f, 0.8f, 0.2f, 1.0f}},
      {CmdType::ENABLE_ADAPTIVE_BEAMFORMING, 2.0f, {0.2f, 0.6f, 1.0f, 1.0f}},
      {CmdType::SET_AGILITY_FREQ, 3.0f, {1.0f, 0.8f, 0.2f, 1.0f}},
      {CmdType::SET_ECCM_REJITTER, 4.0f, {1.0f, 0.5f, 0.2f, 1.0f}},
      {CmdType::SET_ECCM_BURNTHROUGH_GAIN, 5.0f, {1.0f, 0.2f, 0.2f, 1.0f}},
      {CmdType::SET_LPI_POWER, 6.0f, {0.6f, 0.4f, 0.8f, 1.0f}},
  };

  for (const auto& row : rows) {
    std::vector<float> xs;
    std::vector<float> ys;
    for (std::size_t c = 0; c < sim.command_history.size(); ++c) {
      for (const auto& cmd : sim.command_history[c]) {
        if (cmd.type == row.type) {
          xs.push_back(static_cast<float>(c));
          ys.push_back(row.y_pos);
          break;
        }
      }
    }
    if (!xs.empty()) {
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 6.0f,
                                 {row.color.x, row.color.y, row.color.z, row.color.w});
      ImPlot::PlotScatter(CommandTypeName(row.type), xs.data(), ys.data(),
                          static_cast<int>(xs.size()));
    }
  }

  // 干扰强度曲线
  if (!sim.jamming_severity_history.empty()) {
    std::vector<float> cycles(sim.jamming_severity_history.size());
    std::vector<float> scaled(sim.jamming_severity_history.size());
    for (std::size_t i = 0; i < cycles.size(); ++i) {
      cycles[i] = static_cast<float>(i);
      scaled[i] = sim.jamming_severity_history[i] * 7.0f;
    }
    ImPlot::SetNextLineStyle({1.0f, 0.3f, 0.3f, 0.6f}, 1.5f);
    ImPlot::PlotLine("JamSeverity", cycles.data(), scaled.data(), static_cast<int>(cycles.size()));
  }

  ImPlot::EndPlot();
}

// ── 渲染控制 Profile 面板 ──────────────────────────────────────────────────

void RenderControlProfile(const SimState& sim) {
  ImGui::Text("Control Profile");
  ImGui::Separator();

  if (sim.profile_history.empty()) {
    ImGui::TextDisabled("No profile yet.");
    return;
  }

  const auto& prof = sim.profile_history.back();
  auto BoolIndicator = [](const char* label, bool active) {
    if (active) {
      ImGui::TextColored({0.2f, 1.0f, 0.2f, 1.0f}, "[ON]  %s", label);
    } else {
      ImGui::TextColored({0.5f, 0.5f, 0.5f, 1.0f}, "[OFF] %s", label);
    }
  };

  ImGui::Text("Version: %llu", static_cast<unsigned long long>(prof.version));
  ImGui::Spacing();

  ImGui::TextUnformatted("-- LPI --");
  BoolIndicator("Power Control", prof.enable_lpi_power_control);
  if (prof.enable_lpi_power_control) {
    ImGui::SameLine();
    ImGui::Text("(scale=%.2f)", static_cast<double>(prof.lpi_power_scale));
  }
  BoolIndicator("Beamforming", prof.enable_lpi_beamforming);
  ImGui::Text("Dwell Scale: %.2f", static_cast<double>(prof.lpi_dwell_scale));

  ImGui::Spacing();
  ImGui::TextUnformatted("-- ECCM --");
  BoolIndicator("Sidelobe Canceller", prof.enable_sidelobe_canceller);
  BoolIndicator("Adaptive BF", prof.enable_adaptive_beamforming);
  BoolIndicator("Freq Agility", prof.enable_agility_frequency);
  BoolIndicator("Rejitter", prof.enable_eccm_rejitter);
  ImGui::Text("Burnthrough Gain: %.2f", static_cast<double>(prof.eccm_burnthrough_gain));
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
      glfwCreateWindow(1400, 900, "Airborne Radar - ECM Scenario", nullptr, nullptr);
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
    ImGui::TextColored(PhaseColor(sim.current_cycle), "[%s]", PhaseName(sim.current_cycle));
    if (sim.finished) {
      ImGui::SameLine();
      ImGui::TextColored({0.4f, 1.0f, 0.4f, 1.0f}, "Complete.");
    }

    ImGui::Separator();

    // 左右两栏布局
    float total_w = ImGui::GetContentRegionAvail().x;
    float total_h = ImGui::GetContentRegionAvail().y;
    float left_w = total_w * 0.6f;

    // 左侧面板
    ImGui::BeginChild("left", {left_w, 0.0f}, false);
    {
      // 上半：XY 俯视图
      ImGui::BeginChild("xy", {0.0f, total_h * 0.55f}, false);
      RenderXYPlot(sim);
      ImGui::EndChild();

      // 下半：ECCM 指令时间线
      ImGui::BeginChild("cmd_timeline", {0.0f, 0.0f}, false);
      RenderCommandTimeline(sim);
      ImGui::EndChild();
    }
    ImGui::EndChild();

    ImGui::SameLine();

    // 右侧面板
    ImGui::BeginChild("right", {0.0f, 0.0f}, false);
    {
      // 上半：轨迹状态表格
      ImGui::BeginChild("track_table", {0.0f, total_h * 0.5f}, true);
      RenderTrackTable(sim);
      ImGui::EndChild();

      // 下半：控制 Profile
      ImGui::BeginChild("profile", {0.0f, 0.0f}, true);
      RenderControlProfile(sim);
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
