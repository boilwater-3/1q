/**
 * @file SarFocusedImageAssembler.h
 * @brief SAR 输出帧元数据初始化与各处理阶段标记工具函数。
 */

#ifndef ONEQ_SRC_SAR_SESSION_SAR_FOCUSED_IMAGE_ASSEMBLER_H_
#define ONEQ_SRC_SAR_SESSION_SAR_FOCUSED_IMAGE_ASSEMBLER_H_

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

/**
 * @brief 初始化 output_frame 的尺寸、中心斜距与 SNR 占位字段。
 *
 * 在周期执行开始时调用，后续 raw-echo / range-compression / imaging 阶段会逐层覆盖
 * 这些字段。
 * @param[in] config 会话配置，用于推导尺寸与中心斜距。
 * @param[out] frame 待初始化的输出帧。
 */
void InitializeOutputFrameMetadata(const config::SarSessionConfig& config, SarOutputFrame* frame);

/**
 * @brief 标记 raw-echo 阶段完成，写入估算 SNR（dB）。
 * @param[out] frame 输出帧，completed_stage 与 estimated_snr_db 字段被更新。
 * @param[in] estimated_snr_db raw history 估算 SNR（dB）。
 */
void MarkRawEchoStage(SarOutputFrame* frame, double estimated_snr_db);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_FOCUSED_IMAGE_ASSEMBLER_H_
