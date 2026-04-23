// Copyright 2026. All Rights Reserved.
//
// @file ar_detection_range_visualizer.cpp
// @brief ImGui + ImPlot 探测距离与 SNR 场景可视化 Demo。
//
// 仿真场景（4 个目标从远处径向逼近雷达）：
//
//   目标 A (ID=4001)：大型目标，RCS 10.0 m²，起始距离 20km
//   目标 B (ID=4002)：中型目标，RCS 2.0 m²，起始距离 18km
//   目标 C (ID=4003)：小型目标，RCS 0.5 m²，起始距离 16km
//   目标 D (ID=4004)：隐身目标，RCS 0.05 m²，起始距离 14km
//
//   所有目标以 -250 m/s 径向速度逼近，共 40 cycle，dt=0.5s。
//   无干扰，纯物理检测场景，用于观察不同 RCS 目标的探测门限差异。
//
// 可视化面板：
//   左上 — 距离-时间图（4 条距离曲线 + 检测成功标记）
//   左下 — 轨迹生命周期时序（Tentative/Confirmed/Lost 散点）
//   右上 — 轨迹状态表格（含距离、RCS、命中/失配）
//   右下 — 关联质量指标面板

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

constexpr int kMaxCycles = 40;
constexpr float kDtSec = 0.5f;

struct TargetInit {
  std::uint64_t id;
  float start_range_m;
  float rcs;
  float radial_speed;  // m/s，负值表示接近
  const char* label;
};

const TargetInit kTargets[] = {
    {4001, 20000.0f, 10.0f, -250.0f, "Large (10m2)"},
    {4002, 18000.0f, 2.0f, -250.0f, "Medium (2m2)"},
    {4003, 16000.0f, 0.5f, -250.0f, "Small (0.5m2)"},
    {4004, 14000.0f, 0.05f, -250.0f, "Stealth (0.05m2)"},
};
constexpr int kTargetCount = sizeof(kTargets) / sizeof(kTargets[0]);

// ── 颜色 ──────────────────────────────────────────────────────────────────────

const ImVec4 kTargetColors[] = {
    {0.12f, 0.47f, 0.71f, 1.0f},  // 蓝
    {0.17f, 0.63f, 0.17f, 1.0f},  // 绿
    {1.00f, 0.50f, 0.05f, 1.0f},  // 橙
    {0.84f, 0.15f, 0.16f, 1.0f},  // 红
};

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
  int current_cycle{0};
  bool finished{false};

  // 各输入目标的真实距离历史（按 target ID 索引）
  std::map<std::uint64_t, std::vector<float>> true_range_km;

  // 各输出轨迹的距离历史（按 association_key）
  std::map<std::uint64_t, std::vector<float>> track_range_km;
  std::map<std::uint64_t, std::vector<float>> track_cycle;

  // 轨迹状态历史
  std::vector<std::map<std::uint64_t, int>> status_history;

  // 最近一帧
  std::vector<airborne_radar::model::TrackStateSnapshot> latest_tracks;

  // 关联质量
  std::vector<airborne_radar::extension::AssociationQualityMetrics> quality_history;

  // 当前各目标距离
  std::map<std::uint64_t, float> current_range;

  void Reset() {
    current_cycle = 0;
    finished = false;
    true_range_km.clear();
    track_range_km.clear();
    track_cycle.clear();
    status_history.clear();
    latest_tracks.clear();
    quality_history.clear();
    current_range.clear();
    for (int i = 0; i < kTargetCount; ++i) {
      current_range[kTargets[i].id] = kTargets[i].start_range_m;
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

  RadarCycleInput input;
  input.dt_sec = kDtSec;

  // 所有目标沿 X 轴径向逼近（简化为一维距离问题）
  for (int i = 0; i < kTargetCount; ++i) {
    float range = sim.current_range[kTargets[i].id];
    // 目标在 X 轴正方向，Y/Z 略有偏移以区分
    float py = static_cast<float>(i - 1) * 200.0f;
    input.scene.push_back(aq::MakeTargetFromCartesian(
        kTargets[i].id, range, py, 3000.0f, kTargets[i].radial_speed, 0.0f, 0.0f, kTargets[i].rcs));
    sim.true_range_km[kTargets[i].id].push_back(range / 1000.0f);
  }

  auto result = session.StepWithResult(input);

  // 记录输出轨迹
  std::map<std::uint64_t, int> cycle_status;
  for (const auto& snap : result.track_output_frame.tracks) {
    const auto key = snap.state.association_key;
    float range = std::sqrt(snap.state.position_x * snap.state.position_x +
                            snap.state.position_y * snap.state.position_y +
                            snap.state.position_z * snap.state.position_z);
    sim.track_range_km[key].push_back(range / 1000.0f);
    sim.track_cycle[key].push_back(static_cast<float>(sim.current_cycle));
    cycle_status[key] = static_cast<int>(snap.state.status);
  }
  sim.status_history.push_back(cycle_status);
  sim.latest_tracks = result.track_output_frame.tracks;
  sim.quality_history.push_back(result.association_quality_metrics);

  // 推进距离
  for (int i = 0; i < kTargetCount; ++i) {
    float& range = sim.current_range[kTargets[i].id];
    range += kTargets[i].radial_speed * kDtSec;
    if (range < 500.0f) range = 500.0f;
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

// ── 渲染距离-时间图 ────────────────────────────────────────────────────────

void RenderRangeTimePlot(const SimState& sim) {
  if (!ImPlot::BeginPlot("Range vs Cycle (True + Tracked)", ImVec2(-1, -1))) {
    return;
  }
  ImPlot::SetupAxes("Cycle", "Range (km)");
  ImPlot::SetupAxisLimits(ImAxis_X1, 0, kMaxCycles);

  // 真实距离曲线（虚线）
  for (int i = 0; i < kTargetCount; ++i) {
    auto it = sim.true_range_km.find(kTargets[i].id);
    if (it == sim.true_range_km.end() || it->second.empty()) continue;

    std::vector<float> cycles(it->second.size());
    for (std::size_t c = 0; c < cycles.size(); ++c) {
      cycles[c] = static_cast<float>(c);
    }

    const auto& col = kTargetColors[i];
    ImPlot::SetNextLineStyle({col.x, col.y, col.z, 0.4f}, 1.0f);
    char label[64];
    std::snprintf(label, sizeof(label), "%s (true)", kTargets[i].label);
    ImPlot::PlotLine(label, cycles.data(), it->second.data(), static_cast<int>(it->second.size()));
  }

  // 跟踪距离（实线 + 标记）
  for (const auto& kv : sim.track_range_km) {
    const auto key = kv.first;
    const auto& ranges = kv.second;
    const auto& cycles = sim.track_cycle.at(key);
    if (ranges.empty()) continue;

    ImVec4 col = TrackColor(key);
    ImPlot::SetNextLineStyle({col.x, col.y, col.z, col.w}, 2.0f);
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f, {col.x, col.y, col.z, col.w});
    char label[32];
    std::snprintf(label, sizeof(label), "Track %llu", static_cast<unsigned long long>(key));
    ImPlot::PlotLine(label, cycles.data(), ranges.data(), static_cast<int>(ranges.size()));
  }

  ImPlot::EndPlot();
}

// ── 渲染轨迹状态时序图 ──────────────────────────────────────────────────────

void RenderStatusTimeline(const SimState& sim) {
  if (!ImPlot::BeginPlot("Track Lifecycle Timeline", ImVec2(-1, -1))) {
    return;
  }
  ImPlot::SetupAxes("Cycle", "Track");
  ImPlot::SetupAxisLimits(ImAxis_X1, 0, kMaxCycles);

  std::vector<std::uint64_t> all_keys;
  for (const auto& kv : sim.track_range_km) {
    all_keys.push_back(kv.first);
  }

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

// ── 渲染轨迹状态表格 ────────────────────────────────────────────────────────

void RenderTrackTable(const SimState& sim) {
  ImGui::Text("Detection Range Test  Cycle %d / %d", sim.current_cycle, kMaxCycles);
  ImGui::Separator();

  if (ImGui::BeginTable(
          "tracks", 7,
          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
    ImGui::TableSetupColumn("Key");
    ImGui::TableSetupColumn("ExtID");
    ImGui::TableSetupColumn("Status");
    ImGui::TableSetupColumn("Range(km)");
    ImGui::TableSetupColumn("Speed");
    ImGui::TableSetupColumn("Hit");
    ImGui::TableSetupColumn("Miss");
    ImGui::TableHeadersRow();

    for (const auto& snap : sim.latest_tracks) {
      const auto& st = snap.state;
      float range = std::sqrt(st.position_x * st.position_x + st.position_y * st.position_y +
                              st.position_z * st.position_z);
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextColored(TrackColor(st.association_key), "%llu",
                         static_cast<unsigned long long>(st.association_key));
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%llu", static_cast<unsigned long long>(st.external_target_id));
      ImGui::TableSetColumnIndex(2);
      ImGui::TextUnformatted(StatusStr(st.status));
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.2f", static_cast<double>(range / 1000.0f));
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.0f", static_cast<double>(st.speed));
      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%u", st.hit_count);
      ImGui::TableSetColumnIndex(6);
      ImGui::Text("%u", st.miss_count);
    }
    ImGui::EndTable();
  }

  // 输入目标真实距离参考
  ImGui::Spacing();
  ImGui::Text("Input Target Reference:");
  for (int i = 0; i < kTargetCount; ++i) {
    auto it = sim.current_range.find(kTargets[i].id);
    float r = (it != sim.current_range.end()) ? it->second : 0.0f;
    ImGui::TextColored(kTargetColors[i], "  %s: %.2f km", kTargets[i].label,
                       static_cast<double>(r / 1000.0f));
  }
}

// ── 渲染关联质量指标 ────────────────────────────────────────────────────────

void RenderQualityMetrics(const SimState& sim) {
  ImGui::Text("Association Quality");
  ImGui::Separator();

  if (sim.quality_history.empty()) {
    ImGui::TextDisabled("No data yet.");
    return;
  }

  const auto& q = sim.quality_history.back();
  ImGui::Text("Detections:   %zu / %d targets", q.detection_count, kTargetCount);
  ImGui::Text("Matched:      %zu", q.matched_count);
  ImGui::Text("New Tracks:   %zu", q.new_track_count);
  ImGui::Text("Missed:       %zu", q.missed_track_count);
  ImGui::Spacing();

  float mr = q.match_rate;
  ImVec4 mr_col = (mr > 0.8f)   ? ImVec4{0.2f, 1.0f, 0.2f, 1.0f}
                  : (mr > 0.5f) ? ImVec4{1.0f, 0.8f, 0.2f, 1.0f}
                                : ImVec4{1.0f, 0.3f, 0.3f, 1.0f};
  ImGui::TextColored(mr_col, "Match Rate:   %.1f%%", static_cast<double>(mr * 100.0f));
  ImGui::Text("Assoc Stress: %.3f", static_cast<double>(q.association_stress));
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
      glfwCreateWindow(1400, 900, "Airborne Radar - Detection Range Test", nullptr, nullptr);
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
    ImGui::Text("4 targets approaching: RCS 10/2/0.5/0.05 m2");
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
      ImGui::BeginChild("range_plot", {0.0f, total_h * 0.6f}, false);
      RenderRangeTimePlot(sim);
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
      ImGui::BeginChild("table", {0.0f, total_h * 0.6f}, true);
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
