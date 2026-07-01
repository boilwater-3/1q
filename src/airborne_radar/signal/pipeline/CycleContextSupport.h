/**
 * @file CycleContextSupport.h
 * @brief 定义 SignalPipeline 周期缓存初始化与配置同步的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/pipeline/CycleExecutor.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 将 scratch 各字段初始化为当前周期的起始状态。
 * @param input_state  本周期输入目标列表（用于确定 target_count）。
 * @param scratch      待初始化的周期暂存区。
 */
void ResetCycleExecutionScratch(const session::ArSceneTargetList& input_state,
                                CycleExecutionScratch& scratch);

/**
 * @brief 根据 target_count 和噪声标准差重新填充量测协方差矩阵。
 */
void RefreshMeasurementCovariances(
    std::size_t target_count, float kalman_measurement_noise_std,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances);

/**
 * @brief 将运行时配置同步到关联引擎、轨迹滤波器和生命周期管理器。
 * @return 同步成功返回 true；生命周期管理器配置更新失败时返回 false。
 */
bool SyncAssociationAndTrackFilterConfigs(
    const ExecutionConfig& runtime_config, association::DataAssociationEngine* association_engine,
    tracking::TrackFilter* track_filter,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager = nullptr);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_
