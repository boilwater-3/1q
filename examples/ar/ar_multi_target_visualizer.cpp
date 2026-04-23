// Copyright 2026. All Rights Reserved.
//
// @file ar_multi_target_visualizer.cpp
// @brief ImGui + ImPlot 多目标密集跟踪场景可视化 Demo。
//
// 仿真场景（6 个目标，4 种典型航迹模式）：
//
//   编队飞行（Formation）：
//     目标 A (ID=3001)：(8km, 1km, 5km)，速度 (-220, -10, 0) m/s，RCS 1.5 m²
//     目标 B (ID=3002)：(8km, 1.3km, 5km)，速度 (-220, -10, 0) m/s，RCS 1.5 m²
//       — 间距 300m 的紧密编队，考验关联器的分辨能力
//
//   交叉航迹（Crossing）：
//     目标 C (ID=3003)：(10km, -2km, 4km)，速度 (-200, 80, 0) m/s，RCS 2.0 m²
//     目标 D (ID=3004)：(10km, 2km, 4km)，速度 (-200, -80, 0) m/s，RCS 2.0 m²
//       — 两条航迹在约 cycle 12 处交叉，考验航迹交换抑制
//
//   高速机动（Maneuver）：
//     目标 E (ID=3005)：(6km, 0, 3km)，速度 (-300, 0, 0) m/s，RCS 1.0 m²
//       — cycle 10 起开始横向机动（vy 从 0 变至 ±150），考验 Kalman/IMM
//
//   远距微弱（Weak）：
//     目标 F (ID=3006)：(15km, -1km, 8km)，速度 (-150, 20, -3) m/s，RCS 0.3 m²
//       — 小 RCS，远距离，探测断续，考验生命周期管理
//
//   共 25 个 cycle，dt=0.5s，无干扰。
//
// 可视化面板：
//   左上 — 轨迹 XY 俯视图（颜色按 association_key）
//   左下 — 轨迹状态时序图（Tentative/Confirmed/Lost 散点）
//   右   — 轨迹状态表格 + 关联质量指标

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "1q/airborne_radar/config/RadarDetailedSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

namespace {

// ── 仿真参数 ──────────────────────────────────────────────────────────────────

constexpr int kMaxCycles = 25;
constexpr float kDtSec = 0.5f;
constexpr int kManeuverStartCycle = 10;

struct TargetInit {
  std::uint64_t id;
  float px, py, pz;
  float vx, vy, vz;
  float rcs;
  const char* group;
};

const TargetInit kTargets[] = {
    // 编队 A/B
    {3001, 8000.0f, 1000.0f, 5000.0f, -220.0f, -10.0f, 0.0f, 1.5f, "Formation"},
    {3002, 8000.0f, 1300.0f, 5000.0f, -220.0f, -10.0f, 0.0f, 1.5f, "Formation"},
    // 交叉 C/D
    {3003, 10000.0f, -2000.0f, 4000.0f, -200.0f, 80.0f, 0.0f, 2.0f, "Crossing"},
    {3004, 10000.0f, 2000.0f, 4000.0f, -200.0f, -80.0f, 0.0f, 2.0f, "Crossing"},
    // 机动 E
    {3005, 6000.0f, 0.0f, 3000.0f, -300.0f, 0.0f, 0.0f, 1.0f, "Maneuver"},
    // 远距微弱 F
    {3006, 15000.0f, -1000.0f, 8000.0f, -150.0f, 20.0f, -3.0f, 0.3f, "Weak"},
};
constexpr int kTargetCount = sizeof(kTargets) / sizeof(kTargets[0]);

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

  // 轨迹历史
  std::map<std::uint64_t, std::vector<float>> hist_x;
  std::map<std::uint64_t, std::vector<float>> hist_y;

  // 轨迹状态历史（cycle → {association_key → status}）
  std::vector<std::map<std::uint64_t, int>> status_history;

  // 最近一帧轨迹
  std::vector<airborne_radar::model::TrackStateSnapshot> latest_tracks;

  // 关联质量历史
  std::vector<airborne_radar::extension::AssociationQualityMetrics> quality_history;

  bool finished{false};

  void Reset() {
    current_cycle = 0;
    hist_x.clear();
    hist_y.clear();
    status_history.clear();
    latest_tracks.clear();
    quality_history.clear();
    finished = false;
    target_pos.clear();
    for (int i = 0; i < kTargetCount; ++i) {
      const auto& t = kTargets[i];
      target_pos[t.id] = {t.px, t.py, t.pz, t.vx, t.vy, t.vz, t.rcs};
    }
  }
};

// ── 构造 RadarSession ─────────────────────────────────────────────────────────

std::unique_ptr<airborne_radar::session::RadarSession> MakeSession() {
  namespace aq = airborne_radar::common;
  const auto preset = airborne_radar::config::presets::MakeDetectionMissionRadarSessionConfig();

  airborne_radar::environment::EnvironmentDefaultConfig env = preset.environment;
  env.jamming_sensitivity_profile = environment::ResolveJammingSensitivityProfile(5.0f);

  auto session = std::unique_ptr<airborne_radar::session::RadarSession>(
      new airborne_radar::session::RadarSession(airborne_radar::session::RadarSessionFactory::Create(
          airborne_radar::config::RadarDetailedSessionConfigBuilder(preset)
              .Detection()
              .EnablePhysicsDetection(true)
              .WithPeakPowerW(5e6f)
              .WithFrequencyHz(9.3e9f)
              .WithBandwidthHz(10e6f)
              .WithPulseWidthS(20e-6f)
              .WithPrfHz(500.0f)
              .WithMainBeamGainDb(38.0f)
              .WithNoiseFigureDb(3.5f)
              .End()
              .Environment()
              .WithEnvironmentDefault(env)
              .End()
              .Build()));

  airborne_radar::environment::EnvironmentModelConfig env_cfg;
  session->UpdateEnvironmentModelConfig(env_cfg);

  return session;
}

// ── 推进一个 cycle ────────────────────────────────────────────────────────────

void StepOnce(airborne_radar::session::RadarSession& session, SimState& sim) {
  if (sim.finished || sim.current_cycle >= kMaxCycles) {
    sim.finished = true;
    return;
  }

  namespace aq = airborne_radar::common;
  using airborne_radar::session::RadarCycleInput;

  // 机动目标 E：cycle 10 起加横向加速
  auto it = sim.target_pos.find(3005);
  if (it != sim.target_pos.end() && sim.current_cycle >= kManeuverStartCycle) {
    int phase = (sim.current_cycle - kManeuverStartCycle) % 10;
    if (phase < 5) {
      it->second.vy += 30.0f * kDtSec;  // 向正 Y 加速
    } else {
      it->second.vy -= 30.0f * kDtSec;  // 反转向负 Y
    }
  }

  RadarCycleInput input;
  input.dt_sec = kDtSec;
  for (const auto& kv : sim.target_pos) {
    const auto& p = kv.second;
    input.target_features.push_back(
        aq::MakeTargetFromCartesian(kv.first, p.px, p.py, p.pz, p.vx, p.vy, p.vz, p.rcs));
  }

  auto result = session.StepWithResult(input);

  // 追加轨迹历史
  std::map<std::uint64_t, int> cycle_status;
  for (const auto& snap : result.track_output_frame.tracks) {
    const auto key = snap.state.association_key;
    sim.hist_x[key].push_back(snap.state.position_x / 1000.0f);
    sim.hist_y[key].push_back(snap.state.position_y / 1000.0f);
    cycle_status[key] = static_cast<int>(snap.state.status);
  }
  sim.status_history.push_back(cycle_status);
  sim.latest_tracks = result.track_output_frame.tracks;
  sim.quality_history.push_back(result.association_quality_metrics);

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

const char* StatusStr(airborne_radar::model::DecisionTrackStatus s) {
  switch (s) {
    case airborne_radar::model::DecisionTrackStatus::kTentative:
      return "Tentative";
    case airborne_radar::model::DecisionTrackStatus::kConfirmed:
      return "Confirmed";
    case airborne_radar::model::DecisionTrackStatus::kLost:
      return "Lost";
  }
  return "Unknown";
}

ImPlotMarker StatusMarker(airborne_radar::model::DecisionTrackStatus s) {
  switch (s) {
    case airborne_radar::model::DecisionTrackStatus::kTentative:
      return ImPlotMarker_Circle;
    case airborne_radar::model::DecisionTrackStatus::kConfirmed:
      return ImPlotMarker_Square;
    case airborne_radar::model::DecisionTrackStatus::kLost:
      return ImPlotMarker_Cross;
  }
  return ImPlotMarker_Circle;
}

// ── 渲染 XY 俯视图 ──────────────────────────────────────────────────────────

void RenderXYPlot(const SimState& sim) {
  if (!ImPlot::BeginPlot("XY Top-Down View (Multi-Target)", ImVec2(-1, -1), ImPlotFlags_Equal)) {
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
    std::snprintf(label, sizeof(label), "T%llu", static_cast<unsigned long long>(key));
    ImPlot::PlotLine(label, xs.data(), ys.data(), static_cast<int>(xs.size()));

    // 当前位置标记
    ImPlotMarker marker = ImPlotMarker_Circle;
    for (const auto& snap : sim.latest_tracks) {
      if (snap.state.association_key == key) {
        marker = StatusMarker(snap.state.status);
        break;
      }
    }
    ImPlot::SetNextMarkerStyle(marker, 8.0f, {col.x, col.y, col.z, col.w});
    char slabel[48];
    std::snprintf(slabel, sizeof(slabel), "P%llu", static_cast<unsigned long long>(key));
    ImPlot::PlotScatter(slabel, &xs.back(), &ys.back(), 1);
  }

  ImPlot::EndPlot();
}

// ── 渲染轨迹状态时序图 ──────────────────────────────────────────────────────

void RenderStatusTimeline(const SimState& sim) {
  if (!ImPlot::BeginPlot("Track Status Timeline", ImVec2(-1, -1))) {
    return;
  }
  ImPlot::SetupAxes("Cycle", "Track");
  ImPlot::SetupAxisLimits(ImAxis_X1, 0, kMaxCycles);

  // 收集所有出现过的 association_key
  std::vector<std::uint64_t> all_keys;
  for (const auto& kv : sim.hist_x) {
    all_keys.push_back(kv.first);
  }

  // Tentative = 绿色圆点, Confirmed = 蓝色方块, Lost = 红色叉
  struct StatusStyle {
    int status_val;
    ImPlotMarker marker;
    ImVec4 color;
    const char* name;
  };
  const StatusStyle styles[] = {
      {0, ImPlotMarker_Circle, {0.4f, 0.9f, 0.4f, 0.8f}, "Tentative"},
      {1, ImPlotMarker_Square, {0.2f, 0.6f, 1.0f, 0.8f}, "Confirmed"},
      {2, ImPlotMarker_Cross, {1.0f, 0.3f, 0.3f, 0.8f}, "Lost"},
  };

  for (const auto& style : styles) {
    std::vector<float> xs;
    std::vector<float> ys;
    for (std::size_t c = 0; c < sim.status_history.size(); ++c) {
      for (std::size_t k = 0; k < all_keys.size(); ++k) {
        auto it2 = sim.status_history[c].find(all_keys[k]);
        if (it2 != sim.status_history[c].end() && it2->second == style.status_val) {
          xs.push_back(static_cast<float>(c));
          ys.push_back(static_cast<float>(k));
        }
      }
    }
    if (!xs.empty()) {
      ImPlot::SetNextMarkerStyle(style.marker, 5.0f,
                                 {style.color.x, style.color.y, style.color.z, style.color.w});
      ImPlot::PlotScatter(style.name, xs.data(), ys.data(), static_cast<int>(xs.size()));
    }
  }

  ImPlot::EndPlot();
}

// ── 渲染轨迹状态表格 ──────────────────────────────────────────────────────────

void RenderTrackTable(const SimState& sim) {
  ImGui::Text("Track Status  Cycle %d / %d", sim.current_cycle, kMaxCycles);
  if (sim.current_cycle >= kManeuverStartCycle) {
    ImGui::TextColored({1.0f, 0.8f, 0.2f, 1.0f}, "Target E maneuvering");
  }
  ImGui::Separator();

  if (ImGui::BeginTable(
          "tracks", 7,
          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
    ImGui::TableSetupColumn("Key");
    ImGui::TableSetupColumn("ExtID");
    ImGui::TableSetupColumn("Status");
    ImGui::TableSetupColumn("Speed");
    ImGui::TableSetupColumn("Hit");
    ImGui::TableSetupColumn("Miss");
    ImGui::TableSetupColumn("Pos(km)");
    ImGui::TableHeadersRow();

    for (const auto& snap : sim.latest_tracks) {
      const auto& st = snap.state;
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextColored(TrackColor(st.association_key), "%llu",
                         static_cast<unsigned long long>(st.association_key));
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%llu", static_cast<unsigned long long>(st.external_target_id));
      ImGui::TableSetColumnIndex(2);
      ImGui::TextUnformatted(StatusStr(st.status));
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.0f", static_cast<double>(st.speed));
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%u", st.hit_count);
      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%u", st.miss_count);
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("(%.1f, %.1f)", static_cast<double>(st.position_x / 1000.0f),
                  static_cast<double>(st.position_y / 1000.0f));
    }
    ImGui::EndTable();
  }
}

// ── 渲染关联质量指标 ────────────────────────────────────────────────────────

void RenderQualityMetrics(const SimState& sim) {
  ImGui::Text("Association Quality Metrics");
  ImGui::Separator();

  if (sim.quality_history.empty()) {
    ImGui::TextDisabled("No data yet.");
    return;
  }

  const auto& q = sim.quality_history.back();
  ImGui::Text("Prior Tracks:  %zu", q.prior_track_count);
  ImGui::Text("Detections:    %zu", q.detection_count);
  ImGui::Text("Matched:       %zu", q.matched_count);
  ImGui::Text("New Tracks:    %zu", q.new_track_count);
  ImGui::Text("Missed Tracks: %zu", q.missed_track_count);

  ImGui::Spacing();
  // 用颜色条表示 match_rate
  float mr = q.match_rate;
  ImVec4 mr_col = (mr > 0.8f)   ? ImVec4{0.2f, 1.0f, 0.2f, 1.0f}
                  : (mr > 0.5f) ? ImVec4{1.0f, 0.8f, 0.2f, 1.0f}
                                : ImVec4{1.0f, 0.3f, 0.3f, 1.0f};
  ImGui::TextColored(mr_col, "Match Rate:    %.1f%%", static_cast<double>(mr * 100.0f));
  ImGui::Text("New Track Rate:  %.1f%%", static_cast<double>(q.new_track_rate * 100.0f));
  ImGui::Text("Miss Rate:       %.1f%%", static_cast<double>(q.missed_track_rate * 100.0f));

  ImGui::Spacing();
  ImGui::Text("Mean Cost:     %.2f", static_cast<double>(q.mean_match_cost));
  ImGui::Text("P95 Cost:      %.2f", static_cast<double>(q.p95_match_cost));
  ImGui::Text("Assoc Stress:  %.3f", static_cast<double>(q.association_stress));
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
      glfwCreateWindow(1400, 900, "Airborne Radar - Multi-Target Tracking", nullptr, nullptr);
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
    ImGui::Text("6 targets: Formation(2) + Crossing(2) + Maneuver(1) + Weak(1)");
    if (sim.finished) {
      ImGui::SameLine();
      ImGui::TextColored({0.4f, 1.0f, 0.4f, 1.0f}, "Complete.");
    }

    ImGui::Separator();

    float total_w = ImGui::GetContentRegionAvail().x;
    float total_h = ImGui::GetContentRegionAvail().y;
    float left_w = total_w * 0.6f;

    // 左侧面板
    ImGui::BeginChild("left", {left_w, 0.0f}, false);
    {
      ImGui::BeginChild("xy", {0.0f, total_h * 0.6f}, false);
      RenderXYPlot(sim);
      ImGui::EndChild();

      ImGui::BeginChild("timeline", {0.0f, 0.0f}, false);
      RenderStatusTimeline(sim);
      ImGui::EndChild();
    }
    ImGui::EndChild();

    ImGui::SameLine();

    // 右侧面板
    ImGui::BeginChild("right", {0.0f, 0.0f}, false);
    {
      ImGui::BeginChild("table", {0.0f, total_h * 0.55f}, true);
      RenderTrackTable(sim);
      ImGui::EndChild();

      ImGui::BeginChild("quality", {0.0f, 0.0f}, true);
      RenderQualityMetrics(sim);
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
