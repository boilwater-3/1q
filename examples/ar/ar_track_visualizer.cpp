// Copyright 2026. All Rights Reserved.
//
// @file ar_track_visualizer.cpp
// @brief ImGui + ImPlot 轨迹可视化 Demo。
//
// 仿真场景：
//   目标 A (ID=1001)：(50km, 10km, 5km)，速度 (-200, -30, -10) m/s
//   目标 B (ID=1002)：(40km, -15km, 8km)，速度 (-180, 50, -5) m/s
//   共 20 个 cycle，dt=0.5s；cycle 12 起注入噪声干扰。
//
// 交互：
//   [Step]    — 手动推进一个 cycle
//   [Run All] — 跑满 20 cycle
//   [Reset]   — 清空历史，重建 session

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

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"

namespace {

// ── 仿真参数 ──────────────────────────────────────────────────────────────────

constexpr int kMaxCycles = 20;
constexpr float kDtSec = 0.5f;
constexpr int kJamStartCycle = 12;  // 从第 12 cycle 起注入噪声干扰

// 目标初始状态
struct TargetInit {
  std::uint64_t id;
  float px, py, pz;
  float vx, vy, vz;
  float rcs;
};

constexpr TargetInit kTargets[] = {
    {1001, 5000.0f, 1000.0f, 500.0f, -200.0f, -30.0f, -10.0f, 1.0f},
    {1002, 4000.0f, -1500.0f, 800.0f, -180.0f, 50.0f, -5.0f, 1.2f},
};

// ── 颜色辅助 ──────────────────────────────────────────────────────────────────

ImVec4 TrackColor(std::uint64_t association_key) {
  // 用 association_key % 7 索引 ImPlot 默认调色板
  const ImVec4* colors = ImPlot::GetStyle().Colors;
  (void)colors;
  // ImPlot 颜色通过 SetNextLineStyle 注入；此处仅返回近似色用于表格着色
  static const ImVec4 kPalette[] = {
      {0.12f, 0.47f, 0.71f, 1.0f}, {1.00f, 0.50f, 0.05f, 1.0f}, {0.17f, 0.63f, 0.17f, 1.0f},
      {0.84f, 0.15f, 0.16f, 1.0f}, {0.58f, 0.40f, 0.74f, 1.0f}, {0.55f, 0.34f, 0.29f, 1.0f},
      {0.89f, 0.47f, 0.76f, 1.0f},
  };
  return kPalette[association_key % 7];
}

// ── 仿真状态 ──────────────────────────────────────────────────────────────────

struct SimState {
  // 当前周期目标位置（按输入 ID 索引）
  struct TargetPos {
    float px, py, pz;
    float vx, vy, vz;
    float rcs;
  };
  std::map<std::uint64_t, TargetPos> target_pos;

  int current_cycle{0};

  // 轨迹历史（association_key → 各 cycle 坐标/速度）
  std::map<std::uint64_t, std::vector<float>> hist_x;  // km
  std::map<std::uint64_t, std::vector<float>> hist_y;  // km
  std::map<std::uint64_t, std::vector<float>> hist_speed;

  // 最近一帧各轨迹快照（用于状态表格和散点）
  std::vector<airborne_radar::model::DecisionTrackSnapshot> latest_tracks;

  // 仿真是否已完成
  bool finished{false};

  void Reset() {
    current_cycle = 0;
    hist_x.clear();
    hist_y.clear();
    hist_speed.clear();
    latest_tracks.clear();
    finished = false;
    target_pos.clear();
    for (const auto& t : kTargets) {
      target_pos[t.id] = {t.px, t.py, t.pz, t.vx, t.vy, t.vz, t.rcs};
    }
  }
};

// ── 构造 RadarSession ─────────────────────────────────────────────────────────

std::unique_ptr<airborne_radar::session::RadarSession> MakeSession() {
  namespace aq = airborne_radar::common;
  const auto preset = airborne_radar::config::MakeDetectionMissionRadarSessionConfig();

  airborne_radar::config::SignalDetectionConfig detection = preset.detection;
  detection.enable_physics_detection = true;
  detection.transmitter.peak_power_w = 5e6f;
  detection.transmitter.frequency_hz = 9.3e9f;
  detection.transmitter.bandwidth_hz = 10e6f;
  detection.transmitter.pulse_width_s = 20e-6f;
  detection.transmitter.prf_hz = 500.0f;
  detection.antenna.main_beam_gain_db = 38.0f;
  detection.antenna.nominal_az_beamwidth_deg = 3.5f;
  detection.antenna.nominal_el_beamwidth_deg = 3.5f;
  detection.receiver.noise_figure_db = 3.5f;

  airborne_radar::environment::EnvironmentDefaultConfig env = preset.environment_default_config;
  env.jamming_detection_threshold_db = 5.0f;

  auto session = std::unique_ptr<airborne_radar::session::RadarSession>(
      new airborne_radar::session::RadarSession(airborne_radar::session::RadarSessionFactory::Create(
          airborne_radar::config::RadarSessionConfigBuilder(preset)
              .Detection()
              .WithDetection(detection)
              .End()
              .Environment()
              .WithEnvironmentDefault(env)
              .End()
              .Build()));

  // 空对空场景：关闭地面杂波，仅保留热噪声底
  airborne_radar::environment::EnvironmentModelConfig env_cfg;
  env_cfg.clutter_power_db = -200.0f;         // 杂波清零
  env_cfg.base_propagation_loss_db = 0.0f;    // 无额外传播损耗
  env_cfg.atmospheric_attenuation_db = 0.0f;  // 无大气衰减
  env_cfg.terrain_reflection_db = 0.0f;       // 无地形多径
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
  using airborne_radar::environment::EnvironmentSceneBuilder;
  using airborne_radar::environment::JammerEmitterState;
  using airborne_radar::environment::JammingTechnique;

  // 构造输入
  RadarCycleInput input;
  input.dt_sec = kDtSec;
  for (const auto& kv : sim.target_pos) {
    const auto& p = kv.second;
    input.target_features.push_back(
        aq::MakeTargetFromCartesian(kv.first, p.px, p.py, p.pz, p.vx, p.vy, p.vz, p.rcs));
  }

  // cycle 12+ 注入噪声干扰
  airborne_radar::session::RadarCycleResult result;
  if (sim.current_cycle >= kJamStartCycle) {
    JammerEmitterState jammer;
    jammer.technique = JammingTechnique::kNoiseSuppression;
    jammer.power_db = 12.0f;
    jammer.js_db = 8.0f;
    jammer.frequency_overlap_ratio = 0.25f;
    jammer.prf_lock_risk = 0.10f;
    jammer.in_sidelobe = true;
    // 使用空对空低杂波场景，仅叠加干扰源
    airborne_radar::environment::EnvironmentSceneState scene_st;
    scene_st.clutter_power_db = -200.0f;
    scene_st.base_propagation_loss_db = 0.0f;
    scene_st.atmospheric_attenuation_db = 0.0f;
    scene_st.terrain_reflection_db = 0.0f;
    scene_st.jammer_emitters.push_back(jammer);
    const auto scene = scene_st;
    result = session.StepWithResult(input, scene);
  } else {
    result = session.StepWithResult(input);
  }

  // 追加历史
  for (const auto& snap : result.track_output_frame.tracks) {
    const auto key = snap.state.association_key;
    sim.hist_x[key].push_back(snap.state.position_x / 1000.0f);
    sim.hist_y[key].push_back(snap.state.position_y / 1000.0f);
    sim.hist_speed[key].push_back(snap.state.speed);
  }
  sim.latest_tracks = result.track_output_frame.tracks;

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

// ── 渲染左侧轨迹状态表格 ──────────────────────────────────────────────────────

void RenderTrackTable(const SimState& sim) {
  ImGui::Text("轨迹状态  Cycle %d / %d", sim.current_cycle, kMaxCycles);
  ImGui::Separator();

  if (ImGui::BeginTable(
          "tracks", 6,
          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
    ImGui::TableSetupColumn("ID");
    ImGui::TableSetupColumn("Status");
    ImGui::TableSetupColumn("Speed(m/s)");
    ImGui::TableSetupColumn("Hit");
    ImGui::TableSetupColumn("Miss");
    ImGui::TableSetupColumn("Jam");
    ImGui::TableHeadersRow();

    for (const auto& snap : sim.latest_tracks) {
      const auto& st = snap.state;
      ImGui::TableNextRow();

      // ID
      ImGui::TableSetColumnIndex(0);
      ImVec4 col = TrackColor(st.association_key);
      ImGui::TextColored(col, "%llu", static_cast<unsigned long long>(st.association_key));

      // Status
      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(StatusStr(st.status));

      // Speed
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.1f", static_cast<double>(st.speed));

      // Hit
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%u", st.hit_count);

      // Miss
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%u", st.miss_count);

      // Jam
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

// ── 渲染 XY 顶视图 ────────────────────────────────────────────────────────────

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

    // 设置颜色
    ImVec4 col = TrackColor(key);
    ImPlot::SetNextLineStyle({col.x, col.y, col.z, col.w}, 1.5f);
    char label[32];
    std::snprintf(label, sizeof(label), "Track %llu", static_cast<unsigned long long>(key));
    ImPlot::PlotLine(label, xs.data(), ys.data(), static_cast<int>(xs.size()));

    // 当前位置散点（按状态选 Marker）
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

// ── 渲染速度曲线 ──────────────────────────────────────────────────────────────

void RenderSpeedPlot(const SimState& sim) {
  if (!ImPlot::BeginPlot("Speed vs Cycle", ImVec2(-1, -1))) {
    return;
  }
  ImPlot::SetupAxes("Cycle", "Speed (m/s)");

  for (const auto& kv : sim.hist_speed) {
    const auto key = kv.first;
    const auto& speeds = kv.second;
    if (speeds.empty()) continue;

    // 生成 x 轴（cycle 编号）
    std::vector<float> cycles(speeds.size());
    for (std::size_t i = 0; i < cycles.size(); ++i) {
      cycles[i] = static_cast<float>(i + 1);
    }

    ImVec4 col = TrackColor(key);
    ImPlot::SetNextLineStyle({col.x, col.y, col.z, col.w}, 1.5f);
    char label[32];
    std::snprintf(label, sizeof(label), "Track %llu", static_cast<unsigned long long>(key));
    ImPlot::PlotLine(label, cycles.data(), speeds.data(), static_cast<int>(speeds.size()));
  }

  ImPlot::EndPlot();
}

}  // namespace

// ── main ──────────────────────────────────────────────────────────────────────

int main() {
  // GLFW 初始化
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
      glfwCreateWindow(1280, 800, "Airborne Radar - Track Visualizer", nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "GLFW window creation failed\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // ImGui / ImPlot 初始化
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  // 仿真状态
  SimState sim;
  sim.Reset();
  auto session = MakeSession();

  // 主循环
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // 全屏布局
    int display_w = 0, display_h = 0;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({static_cast<float>(display_w), static_cast<float>(display_h)});
    ImGui::Begin("##main", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

    // ── 顶部控制栏 ────────────────────────────────────────────────────────────
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
    if (sim.finished) {
      ImGui::TextColored({0.4f, 1.0f, 0.4f, 1.0f}, "Simulation complete.");
    } else {
      ImGui::Text("Press [Step] or [Run All] to advance.");
    }

    ImGui::Separator();

    // ── 左面板：轨迹状态表格 ─────────────────────────────────────────────────
    ImGui::BeginChild("left_panel", {300.0f, 0.0f}, true);
    RenderTrackTable(sim);
    ImGui::EndChild();

    ImGui::SameLine();

    // ── 右面板：图表 ─────────────────────────────────────────────────────────
    ImGui::BeginChild("right_panel", {0.0f, 0.0f}, false);

    // XY 顶视图（占右侧 60%）
    float right_h = ImGui::GetContentRegionAvail().y;
    ImGui::BeginChild("xy_plot", {0.0f, right_h * 0.6f}, false);
    RenderXYPlot(sim);
    ImGui::EndChild();

    // 速度曲线（剩余 40%）
    ImGui::BeginChild("speed_plot", {0.0f, 0.0f}, false);
    RenderSpeedPlot(sim);
    ImGui::EndChild();

    ImGui::EndChild();  // right_panel

    ImGui::End();  // ##main

    // 渲染
    ImGui::Render();
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  // 清理
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
