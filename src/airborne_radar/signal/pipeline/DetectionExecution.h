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
