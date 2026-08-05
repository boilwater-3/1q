/**
 * @file FusionEngine.cpp
 * @brief FusionEngine 实现：身份键直挂 + 空间/方位/特征关联 + 置信度滑窗融合。
 */

#include "1q/fusion/FusionEngine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nanoflann.hpp>

#include "1q/coordinate/position_transform.h"
#include "common/geometry/BearingCluster.h"

namespace fusion {

namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::LlaPositionDegM;

constexpr std::uint64_t kUnidentifiedKey = 0U;
// 无身份航迹的合成键起点：与调用方身份键（约定 < 2^63）不冲突。
constexpr std::uint64_t kSyntheticKeyBase = (1ULL << 63);

// 单次量测样本（滑窗元素）。
struct MeasurementSample {
  std::uint32_t source_id{0U};
  bool has_position{false};
  LlaPositionDegM position{};
  bool has_bearing{false};
  double bearing_az_deg{0.0};
  double bearing_el_deg{0.0};
  std::vector<double> feature{};
  double verdict{0.0};
  double quality{0.0};
  std::uint64_t cycle{0U};
};

// 航迹（库内状态，业务不可见）。
struct Track {
  std::uint64_t key{0U};
  std::map<std::uint32_t, std::deque<MeasurementSample>> channel_samples{};
  MeasurementSample anchor{};      /**< 最近一次量测（关联锚点） */
  bool has_anchor{false};
  std::size_t missed_cycles{0U};
  std::uint64_t last_update_cycle{0U};
};

double EuclideanDistance(const EcefPositionM& a, const EcefPositionM& b) {
  const double dx = a.x_m - b.x_m;
  const double dy = a.y_m - b.y_m;
  const double dz = a.z_m - b.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double EuclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
  double sum = 0.0;
  const std::size_t n = std::min(a.size(), b.size());
  for (std::size_t k = 0; k < n; ++k) {
    const double d = a[k] - b[k];
    sum += d * d;
  }
  return std::sqrt(sum);
}

// nanoflann 数据适配器：暴露航迹位置锚点（ECEF 三维，double）。
class TrackAnchorAdaptor {
 public:
  explicit TrackAnchorAdaptor(const std::vector<EcefPositionM>& anchors)
      : anchors_(anchors) {}

  inline std::size_t kdtree_get_point_count() const { return anchors_.size(); }

  inline double kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
    if (idx >= anchors_.size() || dim >= 3U) {
      return 0.0;
    }
    if (dim == 0U) {
      return anchors_[idx].x_m;
    }
    if (dim == 1U) {
      return anchors_[idx].y_m;
    }
    return anchors_[idx].z_m;
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }

 private:
  const std::vector<EcefPositionM>& anchors_;  // 非拥有
};

using TrackKdTree =
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, TrackAnchorAdaptor>,
                                        TrackAnchorAdaptor, 3U, std::size_t>;

// 半径搜索（兼容 nanoflann 1.3.x 与 1.5+ 的返回类型差异，同 ESR KdTreeClusterer 先例）。
std::vector<std::size_t> RadiusSearch(const TrackKdTree& index,
                                      const EcefPositionM& query, double radius_m) {
  using KdDistanceType = typename TrackKdTree::DistanceType;
#if defined(NANOFLANN_VERSION) && (NANOFLANN_VERSION >= 0x150)
  using KdIndexType = typename TrackKdTree::IndexType;
  std::vector<nanoflann::ResultItem<KdIndexType, KdDistanceType>> matches;
  nanoflann::SearchParameters search_params;
#else
  using KdIndexType = std::size_t;
  std::vector<std::pair<KdIndexType, KdDistanceType>> matches;
  nanoflann::SearchParams search_params;
#endif
  search_params.sorted = true;
  const KdDistanceType radius_sq = static_cast<KdDistanceType>(radius_m * radius_m);
  const double query_xyz[3] = {query.x_m, query.y_m, query.z_m};
  index.radiusSearch(query_xyz, radius_sq, matches, search_params);
  std::vector<std::size_t> indices;
  indices.reserve(matches.size());
  for (const auto& match : matches) {
    indices.push_back(match.first);
  }
  return indices;
}

}  // namespace

class FusionEngine::Impl {
 public:
  explicit Impl(const FusionConfig& config) : config_(config) {}

  std::vector<FusedTarget> Update(const std::vector<DetectionRecord>& detections,
                                  std::uint64_t cycle) {
    // 本周期先统一递增失跟计数；收到量测的航迹在 AppendSample 中清零。
    for (auto& entry : tracks_) {
      ++entry.second.missed_cycles;
    }

    // 冻结语义：同周期新建航迹不参与本周期后续关联（自下周期起成为候选）。
    // 快照周期起始航迹键集合，关联阶段（含 KD-tree 建树）只考虑快照内航迹。
    std::set<std::uint64_t> cycle_start_keys;
    for (const auto& entry : tracks_) {
      cycle_start_keys.insert(entry.first);
    }

    // ① 身份键直挂：同键探测归并到同航迹（跨源一致性由调用方保证）。
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> keyed;
    std::vector<std::size_t> unidentified;
    for (std::size_t i = 0; i < detections.size(); ++i) {
      if (detections[i].key == kUnidentifiedKey) {
        unidentified.push_back(i);
      } else {
        keyed[detections[i].key].push_back(i);
      }
    }
    for (const auto& group : keyed) {
      auto it = tracks_.find(group.first);
      if (it == tracks_.end()) {
        it = tracks_.emplace(group.first, Track{group.first}).first;
      }
      for (const std::size_t i : group.second) {
        AppendSample(&it->second, detections[i], cycle);
      }
    }

    // ② 无身份探测：空间/方位/特征分层关联。
    AssociateUnidentified(detections, unidentified, cycle, cycle_start_keys);

    // ③ 滑窗裁剪 + 失跟删除。
    PruneWindowsAndAging();

    return BuildOutput();
  }

  void Reset() {
    tracks_.clear();
    synthetic_key_counter_ = 0U;
  }

 private:
  void AppendSample(Track* track, const DetectionRecord& detection, std::uint64_t cycle) {
    MeasurementSample sample;
    sample.source_id = detection.source_id;
    sample.has_position = detection.has_position;
    sample.position = detection.position;
    sample.has_bearing = detection.has_bearing;
    sample.bearing_az_deg = detection.bearing_az_deg;
    sample.bearing_el_deg = detection.bearing_el_deg;
    sample.feature = detection.feature;
    sample.verdict = detection.verdict;
    sample.quality = detection.quality;
    sample.cycle = cycle;
    track->channel_samples[detection.source_id].push_back(std::move(sample));
    track->anchor = track->channel_samples[detection.source_id].back();
    track->has_anchor = true;
    track->missed_cycles = 0U;
    track->last_update_cycle = cycle;
  }

  // 特征门限：未启用（threshold <= 0）或任一侧无特征/维度不一致时不构成约束。
  bool FeatureGatePasses(const DetectionRecord& detection, const Track& track) const {
    if (!(config_.feature_threshold > 0.0) || !track.has_anchor) {
      return true;
    }
    if (detection.feature.empty() || track.anchor.feature.empty()) {
      return true;
    }
    if (detection.feature.size() != track.anchor.feature.size()) {
      return true;  // 异构传感器特征维度不一致：特征门不约束。
    }
    return EuclideanDistance(detection.feature, track.anchor.feature) <=
           config_.feature_threshold;
  }

  // 无身份探测关联：带位置 → KD-tree 半径搜索；仅方位 → 方位相干；皆无 → 仅特征门。
  // 候选航迹限于周期起始快照（同周期新建航迹不参与本周期关联）。
  // 未关联者创建新航迹（合成键）。
  void AssociateUnidentified(const std::vector<DetectionRecord>& detections,
                             const std::vector<std::size_t>& indices,
                             std::uint64_t cycle,
                             const std::set<std::uint64_t>& cycle_start_keys) {
    if (indices.empty()) {
      return;
    }
    std::vector<std::uint64_t> anchor_track_keys;
    std::vector<EcefPositionM> anchors;
    for (const auto& entry : tracks_) {
      if (cycle_start_keys.count(entry.first) == 0U) {
        continue;
      }
      EcefPositionM ecef{};
      if (entry.second.has_anchor && entry.second.anchor.has_position &&
          oneq::coordinate::TryLlaToEcef(entry.second.anchor.position, &ecef)) {
        anchor_track_keys.push_back(entry.first);
        anchors.push_back(ecef);
      }
    }
    TrackAnchorAdaptor adaptor(anchors);
    TrackKdTree tree(3U, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    if (!anchors.empty()) {
      tree.buildIndex();
    }

    for (const std::size_t i : indices) {
      const DetectionRecord& detection = detections[i];
      double best_cost = std::numeric_limits<double>::infinity();
      std::uint64_t best_key = 0U;
      bool position_unusable = false;
      if (detection.has_position) {
        EcefPositionM query{};
        if (oneq::coordinate::TryLlaToEcef(detection.position, &query)) {
          for (const std::size_t m : RadiusSearch(tree, query, config_.position_radius_m)) {
            const std::uint64_t track_key = anchor_track_keys[m];
            if (!FeatureGatePasses(detection, tracks_.at(track_key))) {
              continue;
            }
            const double cost = EuclideanDistance(anchors[m], query);
            if (cost < best_cost) {
              best_cost = cost;
              best_key = track_key;
            }
          }
        } else {
          // 位置量测非法（如 LLA 越界）：降级走方位/特征通道。
          position_unusable = true;
        }
      }
      if (position_unusable || !detection.has_position) {
        if (detection.has_bearing) {
          for (const auto& entry : tracks_) {
            if (cycle_start_keys.count(entry.first) == 0U) {
              continue;
            }
            const Track& track = entry.second;
            if (!track.has_anchor || !track.anchor.has_bearing ||
                !FeatureGatePasses(detection, track)) {
              continue;
            }
            if (oneq::common::geometry::AreBearingsCoherent(
                    detection.bearing_az_deg, detection.bearing_el_deg,
                    track.anchor.bearing_az_deg, track.anchor.bearing_el_deg,
                    config_.bearing_beamwidth_deg)) {
              const double cost = oneq::common::geometry::AzimuthShortestDifferenceDeg(
                  detection.bearing_az_deg, track.anchor.bearing_az_deg);
              if (cost < best_cost) {
                best_cost = cost;
                best_key = entry.first;
              }
            }
          }
        } else {
          // 无位置无方位：取首个通过特征门的航迹。
          for (const auto& entry : tracks_) {
            if (cycle_start_keys.count(entry.first) == 0U) {
              continue;
            }
            if (FeatureGatePasses(detection, entry.second)) {
              best_key = entry.first;
              break;
            }
          }
        }
      }

      if (best_key != 0U) {
        AppendSample(&tracks_.at(best_key), detection, cycle);
      } else {
        const std::uint64_t new_key = kSyntheticKeyBase + synthetic_key_counter_++;
        auto it = tracks_.emplace(new_key, Track{new_key}).first;
        AppendSample(&it->second, detection, cycle);
      }
    }
  }

  void PruneWindowsAndAging() {
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
      for (auto& channel : it->second.channel_samples) {
        auto& samples = channel.second;
        while (samples.size() > config_.window_size) {
          samples.pop_front();
        }
      }
      if (it->second.missed_cycles > config_.max_missed_cycles) {
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // 置信度 = Σ 判决值 × 质量归一化 × 权重（滑窗内精确求和，不归一化，随证据单调累积）。
  double ComputeConfidence(const Track& track) const {
    double confidence = 0.0;
    for (const auto& channel : track.channel_samples) {
      const double weight = SourceWeight(channel.first);
      for (const auto& sample : channel.second) {
        confidence += sample.verdict * sample.quality * weight;
      }
    }
    return confidence;
  }

  double SourceWeight(std::uint32_t source_id) const {
    if (source_id < config_.source_weights.size()) {
      return config_.source_weights[source_id];
    }
    return 1.0;
  }

  std::vector<FusedTarget> BuildOutput() const {
    std::vector<FusedTarget> output;
    output.reserve(tracks_.size());
    for (const auto& entry : tracks_) {
      FusedTarget target;
      target.key = entry.first;
      for (const auto& channel : entry.second.channel_samples) {
        ChannelMeasurement measurement;
        measurement.source_id = channel.first;
        const auto& samples = channel.second;
        measurement.sample_count = samples.size();
        if (!samples.empty()) {
          const MeasurementSample& latest = samples.back();
          measurement.latest_verdict = latest.verdict;
          measurement.latest_quality = latest.quality;
          measurement.has_position = latest.has_position;
          measurement.position = latest.position;
          measurement.has_bearing = latest.has_bearing;
          measurement.bearing_az_deg = latest.bearing_az_deg;
          measurement.bearing_el_deg = latest.bearing_el_deg;
        }
        target.channels.push_back(std::move(measurement));
      }
      target.confidence = ComputeConfidence(entry.second);
      target.last_update_cycle = entry.second.last_update_cycle;
      output.push_back(std::move(target));
    }
    return output;
  }

  FusionConfig config_;
  std::map<std::uint64_t, Track> tracks_{};
  std::uint64_t synthetic_key_counter_{0U};
};

FusionEngine::FusionEngine(const FusionConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

FusionEngine::~FusionEngine() = default;

std::vector<FusedTarget> FusionEngine::Update(
    const std::vector<DetectionRecord>& detections, std::uint64_t cycle) {
  return impl_->Update(detections, cycle);
}

void FusionEngine::Reset() { impl_->Reset(); }

}  // namespace fusion
