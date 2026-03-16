// Copyright 2026. All Rights Reserved.
//
// Description: 信号检测器，封装物理化的回波评估与探测判决。
// 它是 RadarEquations 纯函数与 Pipeline Stage 之间的有状态桥梁。

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_

#include <random>

#include "1q/airborne_radar/common/AntennaPatternConfig.h"
#include "1q/airborne_radar/signal/detection/RadarEquations.h"

namespace airborne_radar {
namespace common {
struct RadarOrientationConfig;
}  // namespace common
}  // namespace airborne_radar

namespace airborne_radar {
namespace signal {
namespace detection {

/// @brief 单目标探测结果。
struct DetectionResult {
  float echo_power_dbw{-300.0f};   ///< 接收回波功率 (dBW)
  float snr_db{-100.0f};           ///< 信噪比 (dB)
  float detection_prob{0.0f};      ///< 检测概率 Pd
  bool detected{false};            ///< 是否达到门限
  float range_error_std_m{0.0f};   ///< 距离测量标准差 (m)
  float angle_error_std_rad{0.0f}; ///< 方位/俯仰合成的等效角度测量标准差 (rad)，供后续量测协方差建模使用
};

/// @brief 目标回波特征上下文。
struct TargetReturn {
  float rcs_m2{0.0f};              ///< 目标 RCS (m²)
  float range_m{0.0f};             ///< 目标到雷达斜距 (m)
  float look_az_deg{0.0f};         ///< 目标相对雷达坐标系的方位角 (deg)
  float look_el_deg{0.0f};         ///< 目标相对雷达坐标系的俯仰角 (deg)
  bool has_look_angles{false};     ///< 是否携带可用于方向图评估的目标角度
  SwerlingModel swerling_type{kSwerling0}; ///< 目标的 Swerling 起伏模型
};

/// @brief 环境噪声上下文。
struct EnvironmentState {
  float propagation_loss_db{0.0f}; ///< 大气传播往返损耗 (dB)
  float clutter_noise_w{0.0f};     ///< 杂波噪声功率 (W)
  float jam_noise_w{0.0f};         ///< 干扰噪声功率 (W)
};

/// @brief SignalDetector 封装物理化的回波评估与探测判决。
///
/// 它组合 RadarEquations 纯函数完成一条完整的物理检测链路：
///   回波功率预算 → SNR 计算 → 检测概率 → 蒙特卡洛判决 → 测量误差评估
///
/// 通过构造函数注入 RadarSystemConfig 配置雷达参数，
/// 热噪声功率底在构造时一次性预计算。
class SignalDetector {
 public:
  /// @brief 使用雷达系统配置构造检测器。
  /// @param config 完整雷达系统参数
  explicit SignalDetector(RadarSystemConfig config);

  /// @brief 对单个目标执行完整检测链。
  /// @param target               目标回波特征上下文
  /// @param env                  环境噪声上下文
  /// @param pulse_count          积累的脉冲数
  /// @param coherent_integration 是否为相参积累
  /// @param orientation_config   可选的雷达方向/控制配置，用于解析指令态波束宽度
  /// @note 若 orientation_config 非空，则先按 commanded_* / nominal_* 规则解析有效波束宽度，
  ///       再计算等效 angle_error_std_rad；该量会被 SignalPipeline 继续用于量测协方差建模。
  /// @return 探测结果
  DetectionResult Detect(const TargetReturn& target,
                         const EnvironmentState& env,
                         int pulse_count = 1,
                         bool coherent_integration = false,
                         const common::RadarOrientationConfig* orientation_config =
                             nullptr);
  /// @brief 设置随机种子（用于确定性回归测试）。
  /// @param seed 随机数种子
  void SetRandomSeed(unsigned int seed);

 private:
  RadarSystemConfig config_;    ///< 雷达系统配置
  float thermal_noise_w_;       ///< 预计算的接收机热噪声底 (W)
  std::mt19937 rng_;            ///< 确定性随机数引擎
};

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_DETECTION_SIGNAL_DETECTOR_H_
