#include "airborne_radar/signal/detection/ArInterferenceObservationResolver.h"

#include <algorithm>
#include <cmath>
#include <tuple>

#include "common/geometry/BearingCluster.h"
#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

constexpr double kRadiansToDegrees = 57.2957795130823208768;

bool LocalFrameIsUsable(const oneq::coordinate::LocalFrameReference& frame) {
  // LlaPositionDegM 无 IsFinite 重载，显式检查有限性。
  const bool lla_finite = std::isfinite(frame.origin_lla.latitude_deg) &&
                          std::isfinite(frame.origin_lla.longitude_deg) &&
                          std::isfinite(frame.origin_lla.altitude_m);
  return lla_finite && oneq::coordinate::IsFinite(frame.frame_attitude_deg);
}

// 把发射 ECEF 位置转换到雷达局部笛卡尔系下的方位/俯仰（与 ArSceneTarget look angle 同系）。
// 失败时返回 false，由调用方决定是否回退到 ECEF 切平面方位。
bool TryEcefPositionToRadarLocalAngles(const oneq::coordinate::EcefPositionM& emission_ecef,
                                       const oneq::coordinate::EcefPositionM& receiver_ecef,
                                       const oneq::coordinate::LocalFrameReference& frame,
                                       double* azimuth_deg, double* elevation_deg) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr) {
    return false;
  }
  // 以接收机 ECEF 位置为相对原点，构造视线的 ENU 分量：先转 ECEF→ENU（绝对位置）再取差，
  // 等价于差向量在 origin_lla 处的 ECEF→ENU（线性近似下一致，且与目标 look angle 同口径）。
  oneq::coordinate::EcefPositionM emission_relative;
  emission_relative.x_m = emission_ecef.x_m;  // TryEcefToEnu 以 origin_lla 为原点，需绝对 ECEF
  emission_relative.y_m = emission_ecef.y_m;
  emission_relative.z_m = emission_ecef.z_m;
  oneq::coordinate::EcefPositionM receiver_absolute;
  receiver_absolute.x_m = receiver_ecef.x_m;
  receiver_absolute.y_m = receiver_ecef.y_m;
  receiver_absolute.z_m = receiver_ecef.z_m;
  oneq::coordinate::EnuPositionM emission_enu;
  oneq::coordinate::EnuPositionM receiver_enu;
  if (!oneq::coordinate::TryEcefToEnu(emission_relative, frame.origin_lla, &emission_enu) ||
      !oneq::coordinate::TryEcefToEnu(receiver_absolute, frame.origin_lla, &receiver_enu)) {
    return false;
  }
  const double east = emission_enu.east_m - receiver_enu.east_m;
  const double north = emission_enu.north_m - receiver_enu.north_m;
  const double up = emission_enu.up_m - receiver_enu.up_m;
  // ENU→雷达局部（扣除平台姿态+挂架角）。
  const oneq::coordinate::Vector3d local =
      oneq::coordinate::RotateEnuToLocal(east, north, up, frame.frame_attitude_deg);
  // 局部系方位/俯仰口径与 TargetLookResolver::Resolve 一致：
  //   az = atan2(local.y, local.x)，el = atan2(local.z, hypot(local.x, local.y))。
  const double horizontal = std::hypot(local.x, local.y);
  *azimuth_deg = std::atan2(local.y, local.x) * kRadiansToDegrees;
  *elevation_deg = std::atan2(local.z, horizontal) * kRadiansToDegrees;
  return true;
}

bool SameIdentity(const oneq::electromagnetics::RfEmissionIdentity& left,
                  const oneq::electromagnetics::RfEmissionIdentity& right) {
  return left.platform_id == right.platform_id && left.equipment_id == right.equipment_id &&
         left.emission_id == right.emission_id;
}

const oneq::electromagnetics::RfSceneEmission* FindEmission(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfEmissionIdentity& identity) {
  for (const auto& emission : scene.emissions) {
    if (SameIdentity(emission.identity, identity)) {
      return &emission;
    }
  }
  return nullptr;
}

double CenterFrequencyHz(const oneq::electromagnetics::RfWaveformSchedule& waveform) {
  if (waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep) {
    return 0.5 * (waveform.sweep_start_frequency_hz + waveform.sweep_stop_frequency_hz);
  }
  return waveform.center_frequency_hz;
}

double ObservableBandwidthHz(const oneq::electromagnetics::RfWaveformSchedule& waveform) {
  if (waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep) {
    return std::fabs(waveform.sweep_stop_frequency_hz - waveform.sweep_start_frequency_hz) +
           waveform.occupied_bandwidth_hz;
  }
  return waveform.occupied_bandwidth_hz;
}

using ObservationSortKey =
    std::tuple<double, double, double, double, double, std::uint8_t, double>;

ObservationSortKey MakeSortKey(const session::ArInterferenceObservation& observation) {
  return std::make_tuple(
      observation.estimated_bearing_azimuth_deg, observation.estimated_bearing_elevation_deg,
      observation.estimated_off_boresight_deg, observation.estimated_center_frequency_hz,
      observation.estimated_bandwidth_hz,
      static_cast<std::uint8_t>(observation.estimated_waveform_kind),
      observation.jammer_to_noise_db);
}

}  // namespace

bool TryResolveArInterferenceObservations(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double thermal_noise_power_w, double jammer_to_noise_gate_db,
    const oneq::coordinate::LocalFrameReference& platform_frame,
    std::vector<session::ArInterferenceObservation>* observations) {
  if (observations == nullptr || !oneq::electromagnetics::TryValidateRfSceneFrame(scene) ||
      !std::isfinite(thermal_noise_power_w) || thermal_noise_power_w <= 0.0 ||
      !std::isfinite(jammer_to_noise_gate_db)) {
    return false;
  }
  const bool frame_usable = LocalFrameIsUsable(platform_frame);

  std::vector<session::ArInterferenceObservation> candidate;
  candidate.reserve(incident_links.size());
  for (const auto& link : incident_links) {
    if (SameIdentity(link.identity, own_emission_identity)) {
      continue;
    }
    if (!std::isfinite(link.received_power_w) || link.received_power_w < 0.0) {
      return false;
    }
    const double jammer_to_noise_linear = link.received_power_w / thermal_noise_power_w;
    if (jammer_to_noise_linear <= 0.0) {
      continue;
    }
    const double jammer_to_noise_db = 10.0 * std::log10(jammer_to_noise_linear);
    if (jammer_to_noise_db < jammer_to_noise_gate_db) {
      continue;
    }
    const oneq::electromagnetics::RfSceneEmission* emission = FindEmission(scene, link.identity);
    if (emission == nullptr) {
      return false;
    }
    const double x = emission->position_ecef_m.x_m - receiver.position_ecef_m.x_m;
    const double y = emission->position_ecef_m.y_m - receiver.position_ecef_m.y_m;
    const double z = emission->position_ecef_m.z_m - receiver.position_ecef_m.z_m;
    const double range_m = std::sqrt(x * x + y * y + z * z);
    if (!std::isfinite(range_m) || range_m <= 0.0) {
      return false;
    }
    session::ArInterferenceObservation observation;
    observation.estimated_slant_range_m = range_m;
    observation.estimated_bearing_azimuth_deg = std::atan2(y, x) * kRadiansToDegrees;
    observation.estimated_bearing_elevation_deg = std::asin(z / range_m) * kRadiansToDegrees;
    // 雷达局部系方位（与目标 look angle 同系）。无可用 pose 时留零，由下游回退并告警。
    if (frame_usable) {
      double local_az_deg = 0.0;
      double local_el_deg = 0.0;
      if (TryEcefPositionToRadarLocalAngles(emission->position_ecef_m, receiver.position_ecef_m,
                                            platform_frame, &local_az_deg, &local_el_deg)) {
        observation.has_local_bearings = true;
        observation.estimated_bearing_azimuth_local_deg = local_az_deg;
        observation.estimated_bearing_elevation_local_deg = local_el_deg;
      }
    }
    const double direction_x = x / range_m;
    const double direction_y = y / range_m;
    const double direction_z = z / range_m;
    // 径向速度：相对速度（发射体-接收机）与视线单位向量的点乘（正值表示远离）。
    // 此前实现仅用发射体 ECEF 速度，未扣除接收机（平台）自身运动，对快速移动平台
    // 系统性偏置距离变化率估计，影响反 VGPO 评分门限的准确性。
    if (oneq::coordinate::IsFinite(emission->velocity_ecef_mps) &&
        oneq::coordinate::IsFinite(receiver.velocity_ecef_mps)) {
      const double rel_vx =
          emission->velocity_ecef_mps.x_mps - receiver.velocity_ecef_mps.x_mps;
      const double rel_vy =
          emission->velocity_ecef_mps.y_mps - receiver.velocity_ecef_mps.y_mps;
      const double rel_vz =
          emission->velocity_ecef_mps.z_mps - receiver.velocity_ecef_mps.z_mps;
      observation.estimated_range_rate_mps =
          rel_vx * direction_x + rel_vy * direction_y + rel_vz * direction_z;
    } else if (oneq::coordinate::IsFinite(emission->velocity_ecef_mps)) {
      // 回退：接收机 ECEF 速度非有限时仅用发射体速度。
      observation.estimated_range_rate_mps =
          emission->velocity_ecef_mps.x_mps * direction_x +
          emission->velocity_ecef_mps.y_mps * direction_y +
          emission->velocity_ecef_mps.z_mps * direction_z;
    }
    const double boresight_dot =
        std::max(-1.0, std::min(1.0, direction_x * receiver.antenna.boresight_ecef.x +
                                        direction_y * receiver.antenna.boresight_ecef.y +
                                        direction_z * receiver.antenna.boresight_ecef.z));
    observation.estimated_off_boresight_deg = std::acos(boresight_dot) * kRadiansToDegrees;
    observation.estimated_center_frequency_hz = CenterFrequencyHz(emission->waveform);
    observation.estimated_bandwidth_hz = ObservableBandwidthHz(emission->waveform);
    observation.estimated_waveform_kind = emission->waveform.kind;
    observation.jammer_to_noise_db = jammer_to_noise_db;
    const double quality_scale = std::sqrt(std::max(1.0, jammer_to_noise_linear));
    observation.bearing_standard_deviation_deg =
        receiver.antenna.half_power_beamwidth_deg / quality_scale;
    observation.frequency_standard_deviation_hz =
        emission->waveform.occupied_bandwidth_hz / (2.0 * quality_scale);
    observation.bandwidth_standard_deviation_hz =
        observation.estimated_bandwidth_hz / quality_scale;
    candidate.push_back(observation);
  }
  std::sort(candidate.begin(), candidate.end(), [](const auto& left, const auto& right) {
    return MakeSortKey(left) < MakeSortKey(right);
  });

  // 欺骗特征提取：检测同方向多脉冲列（疑似假目标）。聚类逻辑复用共享几何工具，
  // 与 ESR 的 ClassifyDeception 保持一致的波束宽度口径（含 1.0 度下限钳制）。
  // 聚类在雷达局部系方位上进行（与目标 look angle 同系）；无可用 pose 时回退 ECEF 方位。
  const double beamwidth_deg = receiver.antenna.half_power_beamwidth_deg;
  const auto is_pulse_train = [&candidate](std::size_t i) {
    return candidate[i].estimated_waveform_kind ==
           oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain;
  };
  const auto azimuth_of = [frame_usable, &candidate](std::size_t i) {
    return frame_usable ? candidate[i].estimated_bearing_azimuth_local_deg
                        : candidate[i].estimated_bearing_azimuth_deg;
  };
  const auto elevation_of = [frame_usable, &candidate](std::size_t i) {
    return frame_usable ? candidate[i].estimated_bearing_elevation_local_deg
                        : candidate[i].estimated_bearing_elevation_deg;
  };
  for (std::size_t i = 0U; i < candidate.size(); ++i) {
    if (!is_pulse_train(i)) {
      continue;
    }
    const std::size_t coherent_count = oneq::common::geometry::CountCoherentNeighbors(
        candidate.size(), is_pulse_train, azimuth_of, elevation_of, beamwidth_deg, i);
    candidate[i].coherent_emission_count = static_cast<std::uint32_t>(coherent_count);
    if (coherent_count >= 2U) {
      candidate[i].deception_class = session::DeceptionClass::kLikelyFalseTarget;
    }
  }

  for (std::size_t index = 0U; index < candidate.size(); ++index) {
    candidate[index].observation_id = static_cast<std::uint64_t>(index + 1U);
  }
  *observations = candidate;
  return true;
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
