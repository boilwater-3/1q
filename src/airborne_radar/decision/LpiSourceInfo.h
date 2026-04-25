/**
 * @file LpiSourceInfo.h
 * @brief 定义供 LPI evaluator 消费的内部威胁来源信息。
 */

#ifndef AIRBORNE_RADAR_DECISION_LPI_SOURCE_INFO_H_
#define AIRBORNE_RADAR_DECISION_LPI_SOURCE_INFO_H_

namespace airborne_radar {
namespace model {

/**
 * @brief LpiSourceInfo 表示供 LPI 模块消费的威胁来源信息摘要。
 *
 * 由 ThreatAssessmentEvaluator 在目标分类过程中提取并填充。
 * 包含雷达低截获概率决策所需的威胁特征：目标类型、距离、速度、RCS 等。
 * 该结构体为决策模块内部传输类型，不对外暴露。
 */
struct LpiSourceInfo {
  bool has_recon_platform{false};         /**< 当前周期目标中是否存在电子侦察飞行器或侦察类设备 */
  float threat_range_km{0.0f};            /**< 最近威胁目标距离（单位：km） */
  float threat_closure_speed_mps{0.0f};   /**< 最近威胁目标接近速度（单位：m/s） */
  float threat_rcs{0.0f};                 /**< 最近威胁目标 RCS（单位：m²） */
  float threat_azimuth_deg{0.0f};         /**< 最近威胁目标方位角（单位：deg，未来扩展预留） */
  float threat_elevation_deg{0.0f};       /**< 最近威胁目标俯仰角（单位：deg，未来扩展预留） */
};

}  // namespace model
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_LPI_SOURCE_INFO_H_
