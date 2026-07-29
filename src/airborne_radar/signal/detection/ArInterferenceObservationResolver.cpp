#include "airborne_radar/signal/detection/ArInterferenceObservationResolver.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <tuple>

#include "airborne_radar/signal/detection/RadarEquations.h"
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

// splitmix32 finalizer keyed by a per-observable domain tag, mirroring the
// ECM/SBIRS DeriveStreamSeed convention. Two observables with the same base
// seed but distinct tags draw from uncorrelated sub-streams, so the range and
// range-rate perturbations are independent.
const std::uint32_t kRangeDomain = UINT32_C(0x524e4745);    // "RNGE"
const std::uint32_t kRangeRateDomain = UINT32_C(0x52415445);  // "RATE"

std::uint32_t DeriveStreamSeed(std::uint32_t base_seed, std::uint32_t domain_tag) {
  std::uint32_t value = base_seed ^ domain_tag;
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return value == 0U ? 1U : value;
}

// Add a deterministic zero-mean Gaussian perturbation of the given standard
// deviation to a truth-derived scalar, so the observable is no longer an exact
// simulation truth value (contract.md:348). The perturbation is keyed by the
// base seed + domain tag + an emission-stable position tag, making it
// reproducible across replays (same inputs ⇒ same output) while decorrelating
// distinct emissions. Returns the perturbed value, or the original on any
// non-finite result (caller still validates downstream).
double Perturb(double truth_value, double std_dev, std::uint32_t base_seed,
               std::uint32_t domain_tag, std::uint64_t emission_tag) {
  if (!std::isfinite(truth_value) || !std::isfinite(std_dev) || std_dev <= 0.0) {
    return truth_value;
  }
  // Mix the emission-stable tag into the seed so two emissions in the same
  // cycle draw independent noise; base seed already encodes cycle + receiver.
  const std::uint32_t seed =
      DeriveStreamSeed(base_seed, domain_tag) ^
      static_cast<std::uint32_t>(emission_tag >> 32U) ^
      static_cast<std::uint32_t>(emission_tag & 0xFFFF'FFFFU);
  std::mt19937 engine(seed == 0U ? 1U : seed);
  std::normal_distribution<double> distribution(0.0, std_dev);
  return truth_value + distribution(engine);
}

}  // namespace

bool TryResolveArInterferenceObservations(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double thermal_noise_power_w, double jammer_to_noise_gate_db,
    const oneq::coordinate::LocalFrameReference& platform_frame,
    std::uint32_t perturbation_seed,
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
    // 发射稳定的 64 位标签：用于把斜距/径向速度扰动与具体发射绑定，使同种子同输入下
    // 可复现、不同发射之间互不相关。组合 platform/equipment/emission 身份（truth 身份
    // 仅用于种子混入，不写入 observation，仍遵守去真值化约定）。
    const std::uint64_t emission_tag =
        (static_cast<std::uint64_t>(link.identity.platform_id) << 48U) |
        (static_cast<std::uint64_t>(link.identity.equipment_id) << 32U) |
        static_cast<std::uint64_t>(link.identity.emission_id);
    // 距离/径向速度测量噪声标准差：以接收端 J/N 作为有效信噪比，发射占用带宽作为距离
    // 分辨带宽。径向速度噪声近似为 σ_v ≈ σ_R / dwell（dwell 取场景窗口时长，下界保护）。
    const double range_std_m =
        static_cast<double>(RadarEquations::ComputeRangeErrorStdDev(
            static_cast<float>(jammer_to_noise_db),
            static_cast<float>(emission->waveform.occupied_bandwidth_hz)));
    const double dwell_s = std::isfinite(scene.window_duration_s) && scene.window_duration_s > 0.0
                               ? scene.window_duration_s
                               : 1.0e-3;
    const double range_rate_std_mps = range_std_m / dwell_s;

    session::ArInterferenceObservation observation;
    // 去真值化：斜距叠加由 MeasurementErrorModel 标准差驱动的确定性零均值噪声，
    // 不再是精确仿真真值（contract.md:348）。
    double perturbed_range_m =
        Perturb(range_m, range_std_m, perturbation_seed, kRangeDomain, emission_tag);
    if (!std::isfinite(perturbed_range_m) || perturbed_range_m <= 0.0) {
      // 扰动后落入非物理区间时退回真值几何并 fail-closed（与既有 range 校验一致）。
      return false;
    }
    observation.estimated_slant_range_m = perturbed_range_m;
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
    double range_rate_mps = 0.0;
    if (oneq::coordinate::IsFinite(emission->velocity_ecef_mps) &&
        oneq::coordinate::IsFinite(receiver.velocity_ecef_mps)) {
      const double rel_vx =
          emission->velocity_ecef_mps.x_mps - receiver.velocity_ecef_mps.x_mps;
      const double rel_vy =
          emission->velocity_ecef_mps.y_mps - receiver.velocity_ecef_mps.y_mps;
      const double rel_vz =
          emission->velocity_ecef_mps.z_mps - receiver.velocity_ecef_mps.z_mps;
      range_rate_mps = rel_vx * direction_x + rel_vy * direction_y + rel_vz * direction_z;
    } else if (oneq::coordinate::IsFinite(emission->velocity_ecef_mps)) {
      // 回退：接收机 ECEF 速度非有限时仅用发射体速度。
      range_rate_mps = emission->velocity_ecef_mps.x_mps * direction_x +
                       emission->velocity_ecef_mps.y_mps * direction_y +
                       emission->velocity_ecef_mps.z_mps * direction_z;
    }
    // 去真值化：径向速度同样叠加确定性噪声（contract.md:348）。
    observation.estimated_range_rate_mps =
        Perturb(range_rate_mps, range_rate_std_mps, perturbation_seed, kRangeRateDomain,
                emission_tag);
    const double boresight_dot =
        std::max(-1.0, std::min(1.0, direction_x * receiver.antenna.boresight_ecef.x +
                                        direction_y * receiver.antenna.boresight_ecef.y +
                                        direction_z * receiver.antenna.boresight_ecef.z));
    observation.estimated_off_boresight_deg = std::acos(boresight_dot) * kRadiansToDegrees;
    observation.estimated_center_frequency_hz = CenterFrequencyHz(emission->waveform);
    observation.estimated_bandwidth_hz = ObservableBandwidthHz(emission->waveform);
    observation.estimated_waveform_kind = emission->waveform.kind;
    observation.jammer_to_noise_db = jammer_to_noise_db;
    // VGPO 物理可观测特征：相对接收机调谐载频（本振中心）的中心频率偏移。VGPO 把转发载频
    // 拖离威胁雷达频率，接收端观测到的偏移即速度波门拖引的直接证据（不依赖 ECM 真值）。
    const double reference_carrier_hz = std::isfinite(receiver.center_frequency_hz) &&
                                                receiver.center_frequency_hz > 0.0
                                            ? receiver.center_frequency_hz
                                            : observation.estimated_center_frequency_hz;
    observation.estimated_carrier_offset_hz =
        observation.estimated_center_frequency_hz - reference_carrier_hz;
    // RGPO 物理可观测特征：首脉冲到达时间相对几何单程传播期望的滞后。发射体按窗口边界
    // 准时发射时，接收首脉冲应在 window_start + range/c 到达；超出部分即人工距离拖引。
    // 仅对 kPulseTrain（具有首脉冲时间）有意义；其他波形保持 0。
    if (emission->waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
      const double c = 299792458.0;
      const double one_way_propagation_s = range_m / c;
      const double first_pulse_relative_s =
          emission->waveform.first_pulse_time_s - scene.window_start_time_s;
      observation.estimated_first_pulse_delay_s =
          first_pulse_relative_s - one_way_propagation_s;
    }
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
