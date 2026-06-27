/**
 * @file SarCycleInputAdapter.h
 * @brief 一步法构建 SarCycleInput，封装脉冲状态坐标转换。
 *
 * 平台/目标采用 LLA+NED 坐标（与 SarPlatformState/SarPointTarget 一致），
 * 由 SarSession 内部 ToLocalPoint 转换。脉冲状态由 SarExternalInputAdapter
 * 从 ECEF/LLA 转换为 scene-center-relative ENU 本地直角坐标后填入 raw_iq。
 * 注意：仅轨迹的适配输出当前不进入成像（见下方 SarCycleInputAdapter 契约）。
 */

#ifndef ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_ADAPTER_H_
#define ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/sar/config/SarMissionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarExternalInputAdapter.h"

namespace sar {
namespace session {

/**
 * @brief SarCycleInput 一步坐标适配器。
 *
 * 调用方提供 LLA 平台/目标与外部脉冲运动学列表，即可获得传入 SarSession 的
 * SarCycleInput。脉冲坐标转换以 mission.scene_center_* 为局部原点。
 *
 * @note external_pulses 为空时，raw_iq 保持默认空值，SarSession 走内部 raw echo
 *       生成路径（而非外部 IQ 路径）。
 * @note 仅提供外部脉冲运动学（无 IQ 样本）时，本适配器把转换后的轨迹写入
 *       `raw_iq.pulse_states`/`pulse_count`。但当前 SAR 内部回波路径会从 config +
 *       platform 重算轨迹（不消费 input.raw_iq.pulse_states），因此**该外部轨迹当前
 *       不进入成像**，仅为未来“外部轨迹覆盖”入口预留。该输出不会触发外部 IQ 路径
 *       （HasExternalRawIq 以 IQ 样本为充要条件）。若需提供外部完整 IQ，请直接构造
 *       含 samples_per_pulse/i_values/q_values 的 SarRawIqFrame。
 */
struct ONEQ_API SarCycleInputAdapter {
  /**
   * @brief 从 LLA 平台/目标与外部脉冲列表一步构建 SarCycleInput。
   * @param[in] platform 平台状态（LLA+NED）。
   * @param[in] targets 点目标列表（LLA）。
   * @param[in] mission 任务配置（读取 scene_center 作为脉冲坐标原点）。
   * @param[in] dt_sec 周期步长（单位：秒）。
   * @param[in] external_pulses 外部脉冲运动学列表；为空则不填 raw_iq。
   * @param[out] output 输出 SarCycleInput；可为 nullptr。
   * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
   * @return 所有脉冲转换成功（或无需转换）返回 true。
   */
  static bool Build(const SarPlatformState& platform,
                    const SarPointTargetList& targets,
                    const config::SarMissionConfig& mission,
                    double dt_sec,
                    const std::vector<SarExternalPulseInput>& external_pulses,
                    SarCycleInput* output,
                    SarCoordinateStatus* status = nullptr);

 private:
  SarCycleInputAdapter() = delete;
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_ADAPTER_H_
