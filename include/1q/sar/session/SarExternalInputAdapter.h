/**
 * @file SarExternalInputAdapter.h
 * @brief 定义 SAR 外部脉冲状态坐标适配接口。
 *
 * SAR 会话内部统一消费 scene-center-relative ENU 本地直角坐标
 * （见 SarCycleInput.h 的 SarRawIqFrame 坐标系约定）。本头提供从
 * 外部 ECEF/LLA 运动学到该本地直角坐标的转换辅助，避免调用方
 * 自行处理坐标变换细节。
 */

#ifndef ONEQ_SAR_SESSION_SAR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_SAR_SESSION_SAR_EXTERNAL_INPUT_ADAPTER_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/sar/config/SarMissionConfig.h"
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
