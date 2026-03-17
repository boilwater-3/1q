// Copyright 2026. All Rights Reserved.
//
// Description: 定义决策层输出的控制意图。

#ifndef AIRBORNE_RADAR_COMMON_CONTROL_DIRECTIVE_H_
#define AIRBORNE_RADAR_COMMON_CONTROL_DIRECTIVE_H_

#include <boost/any.hpp>

namespace airborne_radar {
namespace common {

/// @brief ControlDirectiveSource 表示控制意图来源模块。
enum class ControlDirectiveSource {
  /// @brief 未知来源。
  UNKNOWN = 0,

  /// @brief 威胁评估/分类来源。
  THREAT_ASSESSMENT,

  /// @brief 发射控制来源。
  EMISSION_CONTROL,

  /// @brief 生存性/ECCM 来源。
  SURVIVABILITY,

  /// @brief 旧责任链适配来源。
  LEGACY_PIPELINE
};

/// @brief ControlDirectiveType 表示决策层输出的控制意图类型。
enum class ControlDirectiveType {
  /// @brief 空意图。
  NONE = 0,

  /// @brief 请求降低发射功率。
  REQUEST_LPI_POWER_REDUCTION,

  /// @brief 请求启用 LPI 波束形成。
  REQUEST_LPI_BEAMFORMING,

  /// @brief 请求调整 LPI 驻留参数。
  REQUEST_LPI_DWELL,

  /// @brief 请求启用旁瓣对消。
  REQUEST_ENABLE_SIDELOBE_CANCELLER,

  /// @brief 请求启用自适应波束形成。
  REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,

  /// @brief 请求启用频率捷变。
  REQUEST_AGILITY_FREQUENCY,

  /// @brief 请求启用重频抖动。
  REQUEST_ECCM_REJITTER,

  /// @brief 请求提升烧穿增益。
  REQUEST_ECCM_BURNTHROUGH_GAIN
};

/// @brief ControlDirective 表示一条可带附加信息的控制意图。
struct ControlDirective {
  /// @brief 控制意图类型。
  ControlDirectiveType type{ControlDirectiveType::NONE};

  /// @brief 控制意图来源。
  ControlDirectiveSource source{ControlDirectiveSource::UNKNOWN};

  /// @brief 附加信息。
  boost::any info;

  /// @brief 默认构造。
  ControlDirective() = default;

  /// @brief 便捷构造。
  ControlDirective(ControlDirectiveType directive_type,
                   ControlDirectiveSource directive_source,
                   const boost::any& directive_info = boost::any())
      : type(directive_type),
        source(directive_source),
        info(directive_info) {}
};

} // namespace common
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_COMMON_CONTROL_DIRECTIVE_H_
