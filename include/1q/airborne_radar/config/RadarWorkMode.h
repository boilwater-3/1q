/**
 * @file RadarWorkMode.h
 * @brief 定义机载雷达工作子模式枚举。
 * @note “可外部调整”定义：调用方可在不重建 `RadarSession` 的前提下，通过公开 API 直接提交修改。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_WORK_MODE_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_WORK_MODE_H_

namespace airborne_radar {
namespace common {
namespace config {

/**
 * @brief RadarWorkSubMode 表示机载雷达工作子模式。
 * @note 该枚举对应运行期可外部调整控制量。
 */
enum class RadarWorkSubMode {
  kStby = 0, /**< 待机：波束停泊并停止扫描 */
  kTas = 1,  /**< 目标捕获扫描：较密集重访扫描 */
  kTws = 2,  /**< 边扫边跟踪：常规二维扫描 */
  kStt = 3   /**< 单目标跟踪：固定驻留点 */
};

}  // namespace config
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_WORK_MODE_H_
