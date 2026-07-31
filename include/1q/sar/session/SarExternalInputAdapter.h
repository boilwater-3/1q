/**
 * @file SarExternalInputAdapter.h
 * @brief 定义 SAR 外部脉冲状态坐标适配接口。
 *
 * SAR 会话内部统一消费 scene-center-relative ENU 本地直角坐标
 * （见 SarCycleInput.h 的 SarRawIqFrame 坐标系约定）。本头提供从
 * 外部 ECEF/LLA 运动学到该本地直角坐标的转换辅助，避免调用方
 * 自行处理坐标变换细节。
 *
 * @par 适配范围说明
 * 与 AR/ESR/EOS 三模块的 ExternalInputAdapter 不同，本头**只适配脉冲状态
 * （PulseState），不适配平台/目标**。原因是 SAR 内部坐标结构本就不同：
 * - AR/ESR/EOS 内部平台/目标统一用局部直角 `PoseState`，故其 adapter 必须把
 *   外部 ECEF 平台与目标都转换到局部直角；
 * - SAR 的 `SarPlatformState` / `SarPointTarget` 内部**直接存 LLA**，调用方填 LLA
 *   即可，无需坐标转换；唯独外部 IQ 数据的 `pulse_states` 要求 scene-center
 *   ENU 本地直角（上游系统交付的轨迹坐标系），这一段才是真实痛点。
 * 因此本适配器聚焦脉冲坐标转换，而非照搬三模块的全量 ECEF→局部直角模式。
 */

#ifndef ONEQ_SAR_SESSION_SAR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_SAR_SESSION_SAR_EXTERNAL_INPUT_ADAPTER_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/sar/config/SarMissionConfig.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"

namespace sar {
namespace session {

/**
 * @brief SarCoordinateStatus 表示坐标适配结果的成败与原因。
 */
enum class ONEQ_API SarCoordinateStatus {
  kOk = 0,                   /**< 转换成功 */
  kNullOutput,               /**< 输出指针为空 */
  kCoordinateTransformFail   /**< 输入 NaN/Inf 或坐标变换数值失败 */
};

/**
 * @brief SarExternalPulseInput 表示外部脉冲状态输入（ECEF 或 LLA）。
 *
 * 位置参考系由 `kinematics.position_frame` 决定（kEcef 或 kLla），
 * 速度固定为 ECEF 坐标系（与 oneq::coordinate::ExternalKinematics 约定一致）。
 */
struct ONEQ_API SarExternalPulseInput {
  std::uint64_t pulse_id{0U};                         /**< 脉冲序号 */
  double time_s{0.0};                                 /**< 脉冲时间（单位：s） */
  oneq::coordinate::ExternalKinematics kinematics{};  /**< 外部运动学（位置+速度+姿态） */
};

/**
 * @brief 由 SAR 任务配置的 scene_center 构造本地坐标参考系。
 *
 * 返回的 LocalFrameReference 以 scene_center 为原点、ENU 轴序（frame_attitude 为零），
 * 供 TryMakePulseFromExternalKinematics 使用。
 *
 * @param[in] mission SAR 任务配置（读取 scene_center_*）。
 * @return 以 scene_center 为原点的 ENU 局部参考系。
 */
ONEQ_API oneq::coordinate::LocalFrameReference BuildSceneCenterReference(
    const config::SarMissionConfig& mission);

/**
 * @brief 便捷重载：从 SarSessionConfig 提取 mission 域。
 */
inline oneq::coordinate::LocalFrameReference BuildSceneCenterReference(
    const config::SarSessionConfig& config) {
  return BuildSceneCenterReference(config.mission);
}

/**
 * @brief 将单个外部脉冲运动学转换为 scene-center-relative ENU 本地直角坐标 PulseState。
 *
 * 位置按 `input.kinematics.position_frame` 分支转换：
 * - kEcef：经 TryEcefToEnu；
 * - kLla：经 TryLlaToEnu。
 * 速度经 TryEcefToEnuVelocity（速度固定 ECEF）。所有转换以 `reference.origin_lla`
 * （即 scene_center）为局部原点。
 *
 * @param[in] input 外部脉冲输入。
 * @param[in] reference scene-center 局部参考系（由 BuildSceneCenterReference 构造）。
 * @param[out] pulse 输出 PulseState（scene-center-relative ENU）。
 * @param[out] status 可选，失败原因。
 * @return 转换成功返回 true；输出为空或任一坐标变换失败返回 false。
 */
ONEQ_API bool TryMakePulseFromExternalKinematics(
    const SarExternalPulseInput& input,
    const oneq::coordinate::LocalFrameReference& reference,
    SarRawIqFrame::PulseState* pulse,
    SarCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_EXTERNAL_INPUT_ADAPTER_H_
