/**
 * @file DetectionExecution.h
 * @brief 定义 SignalPipeline 探测执行阶段的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_

#include <cstdint>
#include <vector>

#include "airborne_radar/environment/EnvironmentTypes.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace signal {
namespace detection {
class SignalDetector;
}  // namespace detection
namespace pipeline {

/**
 * @brief 规则 13b 门内归因：SNR 检测门失败主因分类。
 *
 * SNR 检测门（min_snr_db/min_detection_margin_db）折入距离/波束/噪声底/RCS 多种物理
 * 因素；按各因素相对参考状态的损失 dB 判定主因（损失最大者）：参考状态 = 1 km 距离、
 * 主瓣中心增益、1 m² RCS、热噪声底、零传播损耗。传播损耗并入距离项（大气损耗随
 * 距离/仰角耦合，同属链路衰减）。全部损失 <= 0 时返回 kUnknown。
 * @param[in] range_m 目标斜距（m）。
 * @param[in] effective_rcs_m2 目标有效 RCS（m²）。
 * @param[in] one_way_antenna_gain_db 目标方向单程天线增益（dB）。
 * @param[in] main_beam_gain_db 主瓣峰值增益（dB）。
 * @param[in] propagation_loss_db 总传播损耗（dB）。
 * @param[in] total_noise_w 综合噪声底（热噪声+杂波+干扰，W）。
 * @param[in] thermal_noise_w 热噪声底（W）。
 * @return 主因；无法判定为 kUnknown。
 */
session::ArIssueCause ClassifySnrExclusionCause(float range_m, float effective_rcs_m2,
                                                float one_way_antenna_gain_db,
                                                float main_beam_gain_db,
                                                float propagation_loss_db, float total_noise_w,
                                                float thermal_noise_w);

/**
 * @brief 探测执行阶段的输出缓冲区集合（指向外部拥有并按目标索引对齐的若干向量）。
 * @note 各指针由调用方保证非空且容量不小于目标数；本结构不持有所有权。
 */
struct DetectionExecutionBuffers {
  std::vector<detection::ResolvedTargetGeometry>* target_geometry{
      nullptr};                                            /**< 每目标几何解析结果。 */
  std::vector<float>* signal_term_db{nullptr};             /**< 信号项 dB。 */
  std::vector<float>* speed_penalty_db{nullptr};           /**< 速度惩罚项 dB。 */
  std::vector<float>* detection_margin_db{nullptr};        /**< 检测裕量 dB。 */
  std::vector<std::uint8_t>* detection_succeeded{nullptr}; /**< 是否检测成功的逐目标标记。 */
  std::vector<tracking::MeasurementCovariance>* measurement_covariances{
      nullptr}; /**< 逐目标量测协方差。 */
  session::ArIssueList* issues{nullptr}; /**< 规则 13b 排除诊断累积。 */
  std::size_t* excluded_snr_below{nullptr};             /**< 规则 13a SNR 门排除计数。 */
};

/**
 * @brief 执行物理化探测遍历，调用 SignalDetector 完成回波/SNR/检测判决并填充量测协方差。
 * @param[in] input 当前周期场景目标列表。
 * @param[in] config 运行时配置。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @param[in] platform_altitude_m 平台海拔（单位：m），用于目标特定大气损耗计算。
 * @param[in] signal_detector 物理检测器；为 nullptr 时直接返回。
 * @param[in,out] buffers 各目标缓冲区（按索引就地填充）。
 */
bool RunPhysicalDetectionPass(const session::ArSceneTargetList& input,
                              const ExecutionConfig& config,
                              const session::EnvironmentSnapshot& environment_snapshot,
                              float platform_altitude_m,
                              const RfV2DetectionContext* rf_v2_detection_context,
                              detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_
