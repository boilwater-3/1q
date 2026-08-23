/**
 * @file InferenceAcceptanceRecords.cpp
 * @brief 推演验收行：轨迹预报 / 落点 / 发射点 / 发布；关机点按助推-滑行状态机判定。
 */

#include "target_inference/InferenceAcceptanceRecords.h"

#include <cmath>
#include <map>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "common/logging/AcceptanceText.h"
#include "target_inference/InferenceAcceptanceLog.h"

namespace target_inference {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatVec3;

struct LastVel {
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  bool valid{false};
};

std::map<std::uint64_t, LastVel>& LastVelocities() {
  static std::map<std::uint64_t, LastVel> values;
  return values;
}

/**
 * @brief 单航迹关机点判定状态表（会话内累计，仿 LastVelocities 先例）。
 * @note 甲方 2026-08-22 批注「判断其势能+动能最大的地方」：助推-滑行状态机，
 *       双通道助推判别（加速度 > 2.5·μ/r² 或 ε 上涨超 0.1% 势能尺度），
 *       连续 2 拍无助推确认关机；从未助推且能量平稳 3 拍判窗口外。
 *       方法口径与阈值见验收目录条目与 algorithms.md。
 */
std::map<std::uint64_t, BurnoutTrackerState>& BurnoutTrackers() {
  static std::map<std::uint64_t, BurnoutTrackerState> values;
  return values;
}

void HorizontalEllipseFromCov(const std::array<double, 36U>& cov, double* semi_major,
                              double* semi_minor, double* azimuth_deg) {
  // 位置方差在 [x,vx,y,vy,z,vz] 的 (0,0)/(2,2)/(4,4)；水平取 ECEF x-y 近似。
  const double a = cov[0];
  const double b = cov[2];
  const double c = cov[14];
  const double disc = std::sqrt(std::max(0.0, (a - c) * (a - c) + 4.0 * b * b));
  const double l1 = 0.5 * (a + c + disc);
  const double l2 = 0.5 * (a + c - disc);
  *semi_major = std::sqrt(std::max(0.0, l1));
  *semi_minor = std::sqrt(std::max(0.0, l2));
  *azimuth_deg = 0.5 * std::atan2(2.0 * b, a - c) * 57.29577951308232;
}

}  // namespace

double SpecificMechanicalEnergyJPerKg(const oneq::coordinate::EcefPositionM& position,
                                      const std::array<double, 3U>& velocity_ecef_m_per_s,
                                      double earth_mu_m3_per_s2) {
  const double radius = std::sqrt(position.x_m * position.x_m + position.y_m * position.y_m +
                                  position.z_m * position.z_m);
  if (radius <= 0.0) {
    return 0.0;
  }
  const double speed_sq = velocity_ecef_m_per_s[0] * velocity_ecef_m_per_s[0] +
                          velocity_ecef_m_per_s[1] * velocity_ecef_m_per_s[1] +
                          velocity_ecef_m_per_s[2] * velocity_ecef_m_per_s[2];
  return 0.5 * speed_sq - earth_mu_m3_per_s2 / radius;
}

BurnoutPhase UpdateBurnoutTracker(BurnoutTrackerState& state,
                                  const oneq::coordinate::EcefPositionM& position,
                                  const std::array<double, 3U>& velocity_ecef_m_per_s,
                                  double t_sec, double earth_mu_m3_per_s2) {
  // 方法常数（尺度均由 μ/r 推导，不新造物理常数；依据见 algorithms.md「关机点判定」）：
  // 推力典型几倍重力、滤波速度噪声仅几 m/s，2.5 倍重力门两不相扰；逐拍能量门取
  // 噪声尺度 1e-4·μ/r（高于 v·δv 噪声、低于缓升段单拍增量，防把慢推力误判平稳），
  // 累计下降门沿用旧口径 1e-3·μ/r；防抖/确认/窗口拍数为噪声与果断性折中。
  constexpr double kBoostAccelGravityScale = 2.5;
  constexpr double kEnergyStepScale = 1.0e-4;
  constexpr double kEnergyDeclineScale = 1.0e-3;
  constexpr std::uint32_t kRiseDebounceSamples = 2U;
  constexpr std::uint32_t kCoastConfirmSamples = 2U;
  constexpr std::uint32_t kFlatWindowSamples = 5U;

  const double radius = std::sqrt(position.x_m * position.x_m + position.y_m * position.y_m +
                                  position.z_m * position.z_m);
  if (radius <= 0.0) {
    return BurnoutPhase::kObserving;  // 不可用采样不推进状态（调用方以半径守卫另行写行）。
  }
  const double speed = std::sqrt(
      velocity_ecef_m_per_s[0] * velocity_ecef_m_per_s[0] +
      velocity_ecef_m_per_s[1] * velocity_ecef_m_per_s[1] +
      velocity_ecef_m_per_s[2] * velocity_ecef_m_per_s[2]);
  const double energy =
      SpecificMechanicalEnergyJPerKg(position, velocity_ecef_m_per_s, earth_mu_m3_per_s2);
  const double step_margin = kEnergyStepScale * earth_mu_m3_per_s2 / radius;
  const double decline_margin = kEnergyDeclineScale * earth_mu_m3_per_s2 / radius;
  const double gravity_m_per_s2 = earth_mu_m3_per_s2 / (radius * radius);

  // 助推双通道（任一命中即助推采样）：加速度 |Δv|/Δt > 2.5·当地重力；低推力时
  // 推力做正功 ⇒ ε 逐拍上涨超噪声尺度，连续 ≥2 拍防单拍噪声。物理依据：
  // 无阻力滑行 dε/dt = 0 恒平稳，ε 上涨 ⟺ 推力做正功。
  bool boost_sample = false;
  if (state.valid) {
    const double dt_sec = t_sec - state.last_time_sec;
    if (dt_sec > 0.0) {
      const double dv = std::sqrt(
          (velocity_ecef_m_per_s[0] - state.last_velocity_m_per_s[0]) *
              (velocity_ecef_m_per_s[0] - state.last_velocity_m_per_s[0]) +
          (velocity_ecef_m_per_s[1] - state.last_velocity_m_per_s[1]) *
              (velocity_ecef_m_per_s[1] - state.last_velocity_m_per_s[1]) +
          (velocity_ecef_m_per_s[2] - state.last_velocity_m_per_s[2]) *
              (velocity_ecef_m_per_s[2] - state.last_velocity_m_per_s[2]));
      boost_sample = dv / dt_sec > kBoostAccelGravityScale * gravity_m_per_s2;
    }
    const double d_energy = energy - state.last_energy_j_per_kg;
    state.rise_streak = d_energy > step_margin ? state.rise_streak + 1U : 0U;
    if (!boost_sample && state.rise_streak >= kRiseDebounceSamples) {
      boost_sample = true;
    }
  }

  // 再次助推重开（多脉冲/再次加速语义）：已确认遇助推特征则撤销结论、峰值锚点
  // 解冻续推——否则持续缓升目标（如巡航爬升飞机）的关机时刻会随峰值漂移。
  if (boost_sample && state.confirmed) {
    state.confirmed = false;
  }

  // 峰值累计（确认期间冻结，锚点稳定在确认时刻）：助推段 ε 单调升 ⇒
  // 峰值=最后一个助推采样=关机时刻。
  const bool first_sample = !state.valid;
  if (!state.confirmed && (first_sample || energy > state.energy_j_per_kg)) {
    state.energy_j_per_kg = energy;
    state.time_sec = t_sec;
    state.position = position;
    state.radius_m = radius;
    state.speed_mps = speed;
    state.peak_is_first_sample = first_sample;
  }
  if (first_sample) {
    state.first_energy_j_per_kg = energy;  // 平稳带锚点：从未助推时以首采样为基准。
  }

  // 相态推进：助推清零滑行/平稳计数；已见助推按连续滑行计数；从未助推按平稳带
  // （|ε − 首采样 ε| ≤ 噪声尺度）计数——缓升段累计破带即脱离平稳，不误判窗口外。
  if (boost_sample) {
    state.ever_boosted = true;
    state.coast_samples = 0U;
    state.flat_samples = 0U;
  } else if (state.ever_boosted) {
    ++state.coast_samples;
    state.flat_samples = 0U;
  } else {
    state.coast_samples = 0U;
    const bool within_band = std::fabs(energy - state.first_energy_j_per_kg) <= step_margin;
    state.flat_samples = within_band ? state.flat_samples + 1U : 0U;
  }
  // 关机确认双路径（粘性，遇助推重开）：下降沿——ε 自峰值累计降幅超 0.1% 势能
  // 尺度（旧口径，覆盖慢推力缓升后回落）；滑行——见助推后连续 2 拍无助推。
  if (!state.confirmed) {
    if (energy < state.energy_j_per_kg - decline_margin) {
      state.confirmed = true;
    } else if (state.ever_boosted && state.coast_samples >= kCoastConfirmSamples) {
      state.confirmed = true;
    }
  }

  state.last_velocity_m_per_s = velocity_ecef_m_per_s;
  state.last_time_sec = t_sec;
  state.last_energy_j_per_kg = energy;
  state.valid = true;

  // 相态裁决：确认（粘性）> 助推中 > 窗口外 > 观测中。
  if (state.confirmed) {
    return state.peak_is_first_sample ? BurnoutPhase::kBeforeTrackStart : BurnoutPhase::kConfirmed;
  }
  if (state.ever_boosted) {
    return BurnoutPhase::kBoosting;
  }
  if (state.flat_samples >= kFlatWindowSamples) {
    return BurnoutPhase::kBeforeWindow;
  }
  return BurnoutPhase::kObserving;
}

void WriteInferenceAcceptance(const std::vector<InferenceTrackState>& tracks,
                              const std::vector<TargetInferenceResult>& results,
                              const TargetInferenceConfig& config) {
  if (!INFERENCE_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }

  // 关机点预测（2026-08-22 甲方批注「判断其势能+动能最大的地方」）：助推-滑行
  // 状态机逐航迹判定（双通道助推判别，见 UpdateBurnoutTracker）；行内不带方法
  // 标注，公式与阈值见验收目录条目与 algorithms.md。
  if (tracks.empty()) {
    INFERENCE_ACCEPTANCE_ITEM(0.0f, 0U, "关机点预测", "暂无");
  }
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const InferenceTrackState& track = tracks[i];
    const float row_sim_time = static_cast<float>(track.sim_time_sec);
    const std::uint32_t row_cycle = track.input_cycle_index;
    const double radius = std::sqrt(track.position.x_m * track.position.x_m +
                                    track.position.y_m * track.position.y_m +
                                    track.position.z_m * track.position.z_m);
    if (radius <= 0.0) {
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "关机点预测",
                                "目标键=" + std::to_string(track.key) + " 位置不可用");
      continue;
    }
    BurnoutTrackerState& state = BurnoutTrackers()[track.key];
    const BurnoutPhase phase =
        UpdateBurnoutTracker(state, track.position, track.velocity_ecef_m_per_s,
                             track.sim_time_sec, config.earth_mu_m3_per_s2);
    std::string burnout = "目标键=" + std::to_string(track.key);
    switch (phase) {
      case BurnoutPhase::kBoosting:
        burnout += " 状态=助推中 能量峰值=" + FormatF(state.energy_j_per_kg, 0) + "J/kg@" +
                   FormatF(state.time_sec, 1) + "s";
        break;
      case BurnoutPhase::kConfirmed: {
        burnout += " 状态=已确认";
        burnout += " 关机时刻=" + FormatF(state.time_sec, 1) + "s";
        // 关机点输出经纬高（LLA，库内 TryEcefToLla 换算）：与落点/发射点行同风格，
        // 高度即椭球高（m）。
        oneq::coordinate::LlaPositionDegM peak_lla{};
        if (oneq::coordinate::TryEcefToLla(state.position, &peak_lla)) {
          burnout += " 关机点经纬高=" + FormatVec3(peak_lla.latitude_deg,
                                                  peak_lla.longitude_deg,
                                                  peak_lla.altitude_m, 3);
        } else {
          burnout += " 关机点经纬高=无";
        }
        burnout += " 当时速度=" + FormatF(state.speed_mps, 1) + "m/s";
        burnout += " 机械能峰值=" + FormatF(state.energy_j_per_kg, 0) + "J/kg";
        break;
      }
      case BurnoutPhase::kBeforeTrackStart:
        burnout += " 状态=关机点早于跟踪起点 能量自首采样即下降 峰值=" +
                   FormatF(state.energy_j_per_kg, 0) + "J/kg@" + FormatF(state.time_sec, 1) + "s";
        break;
      case BurnoutPhase::kBeforeWindow:
        burnout += " 状态=关机点在观测窗口外 未观测到助推段 能量=" +
                   FormatF(state.energy_j_per_kg, 0) + "J/kg(自首采样平稳)";
        // 发射时刻锚（库内发射点回推已有量）：夹逼关机 ∈ (发射时刻, 跟踪起点)。
        if (i < results.size() && results[i].trajectory.valid &&
            results[i].trajectory.has_launch) {
          burnout += " 预测发射时刻相对t=" +
                     FormatF(results[i].trajectory.launch_time_offset_sec, 1) + "s";
        }
        break;
      case BurnoutPhase::kObserving:
      default:
        burnout += " 状态=观测中 能量峰值=" + FormatF(state.energy_j_per_kg, 0) + "J/kg@" +
                   FormatF(state.time_sec, 1) + "s";
        break;
    }
    INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "关机点预测", burnout);
  }

  for (std::size_t i = 0U; i < results.size(); ++i) {
    const TargetInferenceResult& result = results[i];
    const TrajectoryPrediction& traj = result.trajectory;
    // 行前缀时间：与航迹采样同源（调用方未提供时保持 0，与既有行为一致）。
    const float row_sim_time =
        i < tracks.size() ? static_cast<float>(tracks[i].sim_time_sec) : 0.0f;
    const std::uint32_t row_cycle = i < tracks.size() ? tracks[i].input_cycle_index : 0U;
    if (!traj.valid) {
      continue;
    }
    double horizon0 = 0.0;
    double horizon1 = 0.0;
    if (!traj.waypoints.empty()) {
      horizon0 = traj.waypoints.front().time_offset_sec;
      horizon1 = traj.waypoints.back().time_offset_sec;
    }
    const double sigma = traj.impact_position_sigma_m;
    std::string forecast = "航路点点数=" + std::to_string(traj.waypoints.size());
    forecast += " 预报时段=[" + FormatF(horizon0, 1) + "," + FormatF(horizon1, 1) + "]s";
    if (traj.has_impact) {
      forecast += " 落点经纬高=" + FormatVec3(traj.impact_point.latitude_deg,
                                              traj.impact_point.longitude_deg,
                                              traj.impact_point.altitude_m, 3);
      forecast += " 误差椭圆半长/半短/方位=(" + FormatF(sigma, 1) + "m," + FormatF(sigma, 1) +
                  "m,0°)";
    } else {
      forecast += " 落点经纬高=无";
      forecast += " 误差椭圆半长/半短/方位=无";
    }
    INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "目标轨迹预报", forecast);

    if (traj.has_impact) {
      std::string impact = "初步落点经纬高=" + FormatVec3(traj.impact_point.latitude_deg,
                                                          traj.impact_point.longitude_deg,
                                                          traj.impact_point.altitude_m, 3);
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "落点预测", impact);

      std::string publish = "弹道模型=无推力弹道外推";
      publish += " 预测落点=" + FormatVec3(traj.impact_point.latitude_deg,
                                           traj.impact_point.longitude_deg,
                                           traj.impact_point.altitude_m, 3);
      publish += " 误差1σ=" + FormatF(sigma, 1) + "m";
      publish += " 置信度=0.68";
      // 甲方 2026-08-22 批注「写明文」：库内无标准编解码与发布订阅——封装按
      // 明文报文原样写出（字段自包含），分发状态如实标注明文落盘、未外发。
      publish += " 标准化封装=明文:[落点预报|落点=" +
                 FormatVec3(traj.impact_point.latitude_deg, traj.impact_point.longitude_deg,
                            traj.impact_point.altitude_m, 3) +
                 "|1σ=" + FormatF(sigma, 1) + "m|置信度=0.68]";
      publish += " 分发状态=明文落盘未外发";
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "落点预报与信息发布", publish);
    }

    if (traj.has_launch) {
      double a = 0.0;
      double b = 0.0;
      double az = 0.0;
      HorizontalEllipseFromCov(traj.launch_covariance_ecef, &a, &b, &az);
      std::string launch = "预测发射时刻相对t=" + FormatF(traj.launch_time_offset_sec, 1) + "s";
      launch += " 地理位置=" + FormatVec3(traj.launch_point.latitude_deg, traj.launch_point.longitude_deg,
                                          traj.launch_point.altitude_m, 3);
      launch += " 位置误差1σ=" + FormatF(traj.launch_position_sigma_m, 1) + "m";
      launch += " 椭圆半长/半短/方位=(" + FormatF(a, 1) + "m," + FormatF(b, 1) + "m," +
                FormatF(az, 1) + "°)";
      launch += " 置信度=0.68";
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "发射点预测", launch);
    }

    if (i < tracks.size()) {
      const InferenceTrackState& track = tracks[i];
      LastVel& prev = LastVelocities()[track.key];
      const double speed = std::sqrt(track.velocity_ecef_m_per_s[0] * track.velocity_ecef_m_per_s[0] +
                                     track.velocity_ecef_m_per_s[1] * track.velocity_ecef_m_per_s[1] +
                                     track.velocity_ecef_m_per_s[2] * track.velocity_ecef_m_per_s[2]);
      if (prev.valid) {
        const double dv = std::sqrt(
            (track.velocity_ecef_m_per_s[0] - prev.vx) * (track.velocity_ecef_m_per_s[0] - prev.vx) +
            (track.velocity_ecef_m_per_s[1] - prev.vy) * (track.velocity_ecef_m_per_s[1] - prev.vy) +
            (track.velocity_ecef_m_per_s[2] - prev.vz) * (track.velocity_ecef_m_per_s[2] - prev.vz));
        const double rel = dv / std::max(speed, 1.0);
        if (rel > 0.2) {
          std::string event = "等级=3 目标键=" + std::to_string(track.key);
          event += " 事件=轨迹突变 相对Δv=" + FormatF(rel, 3);
          event += " 位置ECEF=" + FormatVec3(track.position.x_m, track.position.y_m, track.position.z_m, 1);
          INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "特殊事件监测与提示", event);
        }
      }
      prev.vx = track.velocity_ecef_m_per_s[0];
      prev.vy = track.velocity_ecef_m_per_s[1];
      prev.vz = track.velocity_ecef_m_per_s[2];
      prev.valid = true;
    }
  }
}

}  // namespace target_inference
