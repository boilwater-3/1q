/**
 * @file ArEmissionFactory.h
 * @brief 无状态构造器——从 prepare 输入与工程配置编写一次 AR 脉冲列发射。
 *
 * 照搬 ECM 的 EcmWaveformFactory 静态模式：纯函数，不持有状态，不提交 emission ID
 * 或令牌（这些都由 PreparedCycleLedger::CommitPrepared 落定）。失败时仅返回 false，
 * 调用方负责回滚控制器运行期状态。
 */

#ifndef AIRBORNE_RADAR_SESSION_AR_EMISSION_FACTORY_H_
#define AIRBORNE_RADAR_SESSION_AR_EMISSION_FACTORY_H_

#include <cstdint>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/session/ArRfCycleState.h"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 无状态发射构造器：组装 identity/位置/天线/极化/功率预算/脉冲列波形。
 *
 * 调用方先解析 carrier_hz、PRI、pulse_count 与 control_profile，再调用本方法。
 * boresight 解析或波形构造失败时返回 false，且不修改调用方账本状态。
 */
class ArEmissionFactory {
 public:
  /**
   * @brief 编写一次 AR 脉冲列发射。
   *
   * @param input prepare 输入（platform 身份/位置/速度/波束指向/窗口）。
   * @param detection 工程配置（发射机/接收机/天线）。
   * @param control_profile ECCM/LPI 控制档（功率缩放、烧穿增益、rejitter）。
   * @param emission_id 预留的 emission 身份（来自账本计数器，调用方不推进）。
   * @param carrier_hz 已解析的载频。
   * @param pulse_repetition_interval_s 脉冲重复间隔。
   * @param pulse_count 脉冲数。
   * @param timing_seed 去真值化波形时序种子。
   * @param successful_prepare_count 已成功 prepare 次数（时序种子入参）。
   * @param emission 输出发射事实。
   * @return boresight 解析或波形构造成功返回 true；任一失败返回 false。
   */
  static bool TryBuildEmission(const ArPrepareCycleInput& input,
                               const config::engineering::DetectionConfig& detection,
                               const ArControlProfile& control_profile,
                               std::uint64_t emission_id, double carrier_hz,
                               double pulse_repetition_interval_s, std::uint32_t pulse_count,
                               std::uint64_t timing_seed, std::uint64_t successful_prepare_count,
                               oneq::electromagnetics::RfSceneEmission* emission);
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_AR_EMISSION_FACTORY_H_
