#include "airborne_radar/signal/pipeline/DeceptionMeasurementGenerator.h"

#include <cstddef>
#include <cstdint>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

// 合成量测 source_index 的 sentinel：超出任何真实场景列表大小，
// 使下游 per-target 数组边界检查自然跳过，确保合成量测不索引真实 scratch 数据。
constexpr std::size_t kDeceptionSourceIndexSentinel = static_cast<std::size_t>(-1);

}  // namespace

void InjectDeceptionMeasurementsPass(const CycleExecutionContext& context,
                                     CycleExecutionScratch& scratch) {
  // 读取已由关联引擎分配 key 的欺骗候选量测。
  // 候选的 position/velocity/covariance 由 resolver 在生成时根据物理波形参数计算。
  if (context.cycle_input.deception_measurement_candidates.empty() ||
      scratch.deception_candidate_keys.empty()) {
    return;
  }
  const auto& candidates = context.cycle_input.deception_measurement_candidates;
  const auto& keys = scratch.deception_candidate_keys;

  if (keys.size() != candidates.size()) {
    // 中译：欺骗候选键数 {} 与候选数 {} 不匹配。
    // 标识：输入契约校验——键表与候选表不对应时跳过生成，
    //       防止错配生成欺骗量测。
    PROJECT_LOG_ERROR(
        "[DeceptionMeasurementGenerator] candidate_keys size {} mismatch candidates size {}.",
        keys.size(), candidates.size());
    return;
  }

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const std::uint64_t key = keys[i];
    if (key == 0U) {
      continue;  // 未关联（position 无效），跳过。
    }
    const auto& c = candidates[i];
    if (!std::isfinite(c.position.x()) || !std::isfinite(c.position.y()) ||
        !std::isfinite(c.position.z()) || c.position.isZero(1e-6f)) {
      continue;
    }

    tracking::TrackMeasurement m;
    m.raw_measurement.source_index = kDeceptionSourceIndexSentinel;
    m.raw_measurement.target_name = "deception";
    m.raw_measurement.external_target_id = 0U;
    m.raw_measurement.association_key = key;
    m.raw_measurement.matched_existing_track = false;
    m.raw_measurement.classified_as_false_target = true;
    m.raw_measurement.position = c.position;
    m.raw_measurement.measurement_covariance = c.measurement_covariance;
    m.filtered_feature.velocity = c.velocity;
    m.filtered_feature.observed_speed = c.velocity.norm();
    m.filtered_feature.rcs = 0.0f;
    scratch.track_measurements.push_back(m);
  }
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
