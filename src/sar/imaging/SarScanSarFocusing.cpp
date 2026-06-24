#include "sar/imaging/SarScanSarFocusing.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sar {
namespace imaging {

namespace {

// 校验单 burst 区间有效:半开区间、至少 2 脉冲、不越界。
bool IsValidBurstRange(const ScanBurstPulseRange& range, std::size_t total_pulses) {
  return range.start_pulse < range.end_pulse && range.end_pulse <= total_pulses &&
         range.end_pulse - range.start_pulse >= 2U;
}

// 校验 Omega-K 子带配置(burst 脉冲数需覆盖该子带 Omega-KConfig 的 azimuth_pulse_count)。
bool IsValidSubswathConfig(const ScanSarSubswathConfig& subswath, std::size_t total_pulses,
                           std::size_t range_samples) {
  if (subswath.burst_ranges.empty()) {
    return false;
  }
  for (const ScanBurstPulseRange& range : subswath.burst_ranges) {
    if (!IsValidBurstRange(range, total_pulses)) {
      return false;
    }
  }
  // range_sample_count 与 raw history 列数一致即可(各 burst 共享距离维)。
  return subswath.omega_k.range_sample_count == range_samples;
}

// 从 raw history 抽出 burst 子矩阵:[start_pulse, end_pulse) 行。
bool ExtractBurstSubMatrix(const signal::ComplexMatrix& raw, std::size_t start_pulse,
                           std::size_t end_pulse, signal::ComplexMatrix* burst_raw) {
  if (burst_raw == nullptr) {
    return false;
  }
  const std::size_t burst_rows = end_pulse - start_pulse;
  burst_raw->rows = burst_rows;
  burst_raw->cols = raw.cols;
  burst_raw->values.assign(burst_rows * raw.cols, signal::ComplexSample(0.0, 0.0));
  for (std::size_t r = 0U; r < burst_rows; ++r) {
    for (std::size_t c = 0U; c < raw.cols; ++c) {
      (*burst_raw)(r, c) = raw(start_pulse + r, c);
    }
  }
  return true;
}

// 单 burst 聚焦:调条带编排器(broadside,offset=0,等价条带聚焦)。
bool FocusOneBurst(const OmegaKConfig& base_config, std::size_t burst_pulse_count,
                   const signal::ComplexMatrix& burst_raw, FocusedOmegaKImage* burst_output) {
  OmegaKConfig burst_config = base_config;
  burst_config.azimuth_pulse_count = burst_pulse_count;  // burst 实际脉冲数
  // reference_range_m 沿用子带中心(契约 §3.5)。
  return FocusStripmapOmegaK(burst_config, burst_raw, burst_output);
}

// 单个已聚焦 burst 的方位坐标映射信息。
// burst 覆盖脉冲区间 [start_pulse, end_pulse),聚焦图像有 focused_rows 行。
// 映射:burst 图像第 r 行(0-based)对应统一方位轴上的浮点位置
//       azimuth_pos = start_pulse + r * (end_pulse - start_pulse) / focused_rows
struct BurstAzimuthMapping {
  ScanBurstPulseRange range{};
  std::size_t focused_rows{0U};
  signal::ComplexMatrix image{};
};

// 统一方位轴:覆盖所有 burst 的脉冲区间,分辨率取 max(burst focused_rows / burst_pulse_span)
// 的近似,确保不丢失 burst 图像细节。轴长度 = 所有 burst 合并覆盖的脉冲区间长度。
struct UnifiedAzimuthAxis {
  std::size_t start_pulse{0U};
  std::size_t total_pulses{0U};  // 合并覆盖区间的脉冲数
  std::size_t row_count{0U};     // 统一轴行数
  double pulses_per_row{0.0};    // 每行代表的脉冲数
};

// 计算所有 burst 的合并覆盖区间[start_min, end_max)与统一轴分辨率。
UnifiedAzimuthAxis ComputeUnifiedAxis(const std::vector<BurstAzimuthMapping>& bursts) {
  UnifiedAzimuthAxis axis;
  if (bursts.empty()) {
    return axis;
  }
  std::size_t start_min = bursts.front().range.start_pulse;
  std::size_t end_max = bursts.front().range.end_pulse;
  for (const BurstAzimuthMapping& b : bursts) {
    start_min = std::min(start_min, b.range.start_pulse);
    end_max = std::max(end_max, b.range.end_pulse);
  }
  axis.start_pulse = start_min;
  axis.total_pulses = end_max - start_min;

  // 分辨率:每行代表的脉冲数 = min(burst 的脉冲/行比),保证最细 burst 不被降采样。
  // 对每个 burst,pulse_per_row = span / focused_rows。取最小(最细)→ row 最多。
  double finest_pulses_per_row = std::numeric_limits<double>::max();
  for (const BurstAzimuthMapping& b : bursts) {
    if (b.focused_rows > 0U) {
      const double span = static_cast<double>(b.range.end_pulse - b.range.start_pulse);
      finest_pulses_per_row = std::min(finest_pulses_per_row, span / b.focused_rows);
    }
  }
  if (!(finest_pulses_per_row > 0.0) || !std::isfinite(finest_pulses_per_row)) {
    finest_pulses_per_row = 1.0;  // 退化为每脉冲一行
  }
  axis.pulses_per_row = finest_pulses_per_row;
  axis.row_count =
      axis.total_pulses > 0U
          ? static_cast<std::size_t>(std::ceil(static_cast<double>(axis.total_pulses) /
                                               finest_pulses_per_row))
          : 0U;
  if (axis.row_count == 0U) {
    axis.row_count = 1U;
  }
  return axis;
}

// 把 burst 图像第 r 行映射到统一轴浮点位置(脉冲坐标,相对 start_min)。
double BurstRowToAxisPulse(const BurstAzimuthMapping& burst, std::size_t row,
                           std::size_t axis_origin_pulse) {
  const double span = static_cast<double>(burst.range.end_pulse - burst.range.start_pulse);
  const double frac =
      burst.focused_rows > 1U ? static_cast<double>(row) / static_cast<double>(burst.focused_rows - 1U)
                              : 0.0;
  // burst 行 r → 脉冲坐标 = start + frac * (span - 1)(端点对齐到首末脉冲)。
  const double pulse_coord =
      static_cast<double>(burst.range.start_pulse) + frac * (span - 1.0);
  return pulse_coord - static_cast<double>(axis_origin_pulse);
}

// 检测 burst 间是否有脉冲区间重叠(按 start_pulse 升序)。
bool BurstsHaveOverlap(const std::vector<BurstAzimuthMapping>& bursts) {
  for (std::size_t i = 1U; i < bursts.size(); ++i) {
    // 相邻 burst 重叠:b[i-1].end > b[i].start。
    if (bursts[i - 1U].range.end_pulse > bursts[i].range.start_pulse) {
      return true;
    }
  }
  return false;
}

// 非重叠简单堆叠:各 burst 子图像按脉冲顺序纵向堆叠,逐样本保留(单 burst = 条带子集不变量)。
bool MosaicBurstsConcatenate(const std::vector<BurstAzimuthMapping>& bursts,
                             std::size_t range_cols, signal::ComplexMatrix* mosaicked,
                             std::vector<std::size_t>* azimuth_offsets) {
  if (mosaicked == nullptr || azimuth_offsets == nullptr || bursts.empty()) {
    return false;
  }
  azimuth_offsets->clear();
  azimuth_offsets->reserve(bursts.size());
  std::size_t total_rows = 0U;
  for (const BurstAzimuthMapping& burst : bursts) {
    if (burst.image.cols != range_cols) {
      return false;
    }
    total_rows += burst.focused_rows;
  }
  mosaicked->rows = total_rows;
  mosaicked->cols = range_cols;
  mosaicked->values.assign(total_rows * range_cols, signal::ComplexSample(0.0, 0.0));
  std::size_t row_cursor = 0U;
  for (const BurstAzimuthMapping& burst : bursts) {
    azimuth_offsets->push_back(row_cursor);
    for (std::size_t r = 0U; r < burst.focused_rows; ++r) {
      for (std::size_t c = 0U; c < range_cols; ++c) {
        (*mosaicked)(row_cursor + r, c) = burst.image(r, c);
      }
    }
    row_cursor += burst.focused_rows;
  }
  return true;
}

// 加权平均拼接(契约 §4.3):对统一轴每一行,收集所有覆盖该行的 burst 贡献,
// 重叠区按贡献数做幅度平均(非相干);非重叠区直接取唯一贡献。
// 为保证确定性且无插值误差,burst 行映射到最近的统一轴行(最近邻)。
bool MosaicBurstsWithWeightedAverage(const std::vector<BurstAzimuthMapping>& bursts,
                                     std::size_t range_cols,
                                     signal::ComplexMatrix* mosaicked,
                                     std::vector<std::size_t>* azimuth_offsets) {
  if (mosaicked == nullptr || azimuth_offsets == nullptr || bursts.empty()) {
    return false;
  }
  const UnifiedAzimuthAxis axis = ComputeUnifiedAxis(bursts);
  if (axis.row_count == 0U) {
    return false;
  }

  // 累加器:每行每列求和 + 每行贡献计数(用于平均)。
  mosaicked->rows = axis.row_count;
  mosaicked->cols = range_cols;
  mosaicked->values.assign(axis.row_count * range_cols, signal::ComplexSample(0.0, 0.0));
  std::vector<std::size_t> contribution_counts(axis.row_count, 0U);

  for (const BurstAzimuthMapping& burst : bursts) {
    if (burst.focused_rows == 0U || burst.image.cols != range_cols) {
      return false;
    }
    for (std::size_t r = 0U; r < burst.focused_rows; ++r) {
      const double pulse_f = BurstRowToAxisPulse(burst, r, axis.start_pulse);
      // 最近邻映射到统一轴行。
      const double row_f = pulse_f / axis.pulses_per_row;
      if (!std::isfinite(row_f)) {
        continue;
      }
      long signed_row = static_cast<long>(std::llround(row_f));
      if (signed_row < 0 || static_cast<std::size_t>(signed_row) >= axis.row_count) {
        continue;  // 超出统一轴(边沿抖动),跳过
      }
      const std::size_t target_row = static_cast<std::size_t>(signed_row);
      for (std::size_t c = 0U; c < range_cols; ++c) {
        (*mosaicked)(target_row, c) += burst.image(r, c);
      }
      contribution_counts[target_row] += 1U;
    }
  }

  // 幅度平均:重叠区多 burst 贡献 → 除以贡献数(非相干平均)。
  for (std::size_t r = 0U; r < axis.row_count; ++r) {
    if (contribution_counts[r] > 1U) {
      const double inv = 1.0 / static_cast<double>(contribution_counts[r]);
      for (std::size_t c = 0U; c < range_cols; ++c) {
        (*mosaicked)(r, c) *= inv;
      }
    }
  }

  // 记录各 burst 在统一轴中的起始行偏移(供诊断)。
  azimuth_offsets->clear();
  azimuth_offsets->reserve(bursts.size());
  for (const BurstAzimuthMapping& burst : bursts) {
    const double start_pulse_f =
        static_cast<double>(burst.range.start_pulse) - static_cast<double>(axis.start_pulse);
    const std::size_t offset = static_cast<std::size_t>(
        std::max(0.0, std::round(start_pulse_f / axis.pulses_per_row)));
    azimuth_offsets->push_back(offset);
  }
  return true;
}

}  // namespace

bool FocusScanSarOmegaK(const ScanSarFocusConfig& config,
                        const signal::ComplexMatrix& raw_pulse_history,
                        FocusedScanSarImage* output) {
  if (output == nullptr) {
    return false;
  }
  *output = FocusedScanSarImage{};

  // 全局校验:raw history 非空、配置非空。
  if (raw_pulse_history.rows < 2U || raw_pulse_history.cols < 2U ||
      raw_pulse_history.values.size() != raw_pulse_history.rows * raw_pulse_history.cols) {
    output->failure_stage = "raw_history";
    return false;
  }
  if (config.subswaths.empty()) {
    output->failure_stage = "config";
    return false;
  }

  const std::size_t total_pulses = raw_pulse_history.rows;
  const std::size_t range_samples = raw_pulse_history.cols;

  output->subswaths.reserve(config.subswaths.size());
  for (std::size_t sw = 0U; sw < config.subswaths.size(); ++sw) {
    const ScanSarSubswathConfig& subswath_config = config.subswaths[sw];
    if (!IsValidSubswathConfig(subswath_config, total_pulses, range_samples)) {
      output->failure_stage = "subswath_config";
      return false;
    }

    // 阶段 1:逐 burst 聚焦,收集 burst 图像 + 方位映射。
    std::vector<BurstAzimuthMapping> bursts;
    bursts.reserve(subswath_config.burst_ranges.size());
    FocusedScanSarSubswath subswath_result;
    subswath_result.burst_diagnostics.reserve(subswath_config.burst_ranges.size());
    for (const ScanBurstPulseRange& range : subswath_config.burst_ranges) {
      signal::ComplexMatrix burst_raw;
      if (!ExtractBurstSubMatrix(raw_pulse_history, range.start_pulse, range.end_pulse,
                                 &burst_raw)) {
        output->failure_stage = "burst_extract";
        return false;
      }

      FocusedOmegaKImage burst_output;
      if (!FocusOneBurst(subswath_config.omega_k, range.end_pulse - range.start_pulse, burst_raw,
                         &burst_output)) {
        output->failure_stage = "burst_focus";
        return false;
      }
      if (burst_output.diagnostics.failure_stage != "none") {
        output->failure_stage = "burst_focus";
        return false;
      }
      subswath_result.burst_diagnostics.push_back(burst_output.diagnostics);

      BurstAzimuthMapping mapping;
      mapping.range = range;
      mapping.focused_rows = burst_output.image.rows;
      mapping.image = burst_output.image;
      bursts.push_back(std::move(mapping));
    }

    // 各 burst 聚焦图像距离维列数(Omega-K 网格收缩后,可能 < raw cols;同一子带各 burst
    // 走相同网格收缩 → 列数一致)。
    const std::size_t focused_cols = bursts.front().image.cols;

    // 阶段 2:拼接(契约 §4.3)。burst 不重叠时走简单堆叠(逐样本等价,单 burst = 条带子集
    // 不变量);burst 重叠时走加权平均(重叠区多 burst 贡献幅度平均)。
    const bool has_overlap = BurstsHaveOverlap(bursts);
    const bool mosaic_ok = has_overlap
                               ? MosaicBurstsWithWeightedAverage(bursts, focused_cols,
                                                                 &subswath_result.image,
                                                                 &subswath_result.burst_azimuth_offsets)
                               : MosaicBurstsConcatenate(bursts, focused_cols,
                                                          &subswath_result.image,
                                                          &subswath_result.burst_azimuth_offsets);
    if (!mosaic_ok) {
      output->failure_stage = "burst_mosaic";
      return false;
    }

    // 最终校验:子带图像有限。
    for (const signal::ComplexSample& s : subswath_result.image.values) {
      if (!std::isfinite(s.real()) || !std::isfinite(s.imag())) {
        output->failure_stage = "non_finite";
        return false;
      }
    }
    subswath_result.valid = true;
    output->subswaths.push_back(std::move(subswath_result));
  }

  output->failure_stage = "none";
  return true;
}

}  // namespace imaging
}  // namespace sar
