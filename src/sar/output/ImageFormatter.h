/**
 * @file ImageFormatter.h
 * @brief SAR 聚焦图像二进制与 sidecar manifest 输出。
 */

#ifndef ONEQ_SRC_SAR_OUTPUT_IMAGE_FORMATTER_H_
#define ONEQ_SRC_SAR_OUTPUT_IMAGE_FORMATTER_H_

#include <cstdint>
#include <string>

#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace output {

/**
 * @brief 图像输出元数据。
 */
struct ImageOutputMetadata {
  double center_slant_range_m{0.0};
  double estimated_snr_db{0.0};
  std::uint32_t range_sample_count{0U};
  std::uint32_t azimuth_pulse_count{0U};
  std::string source{};
  double origin_lat_deg{0.0};
  double origin_lon_deg{0.0};
  double range_pixel_spacing_m{0.0};
  double azimuth_pixel_spacing_m{0.0};
};

/**
 * @brief 写出 1QSAR v1 二进制格式图像。
 *
 * 格式: magic "1QSAR\x01\x00" (6 bytes)
 *       + uint32_le rows
 *       + uint32_le cols
 *       + float32 real_0 ... real_{N-1}
 *       + float32 imag_0 ... imag_{N-1}
 *
 * 若 image.is_placeholder 为 true 或尺寸为零, 不做任何写盘并返回 true(安全跳过)。
 *
 * @return 成功/文件已写入时返回 true。
 */
bool WriteBinaryImage(const ::sar::session::SarFocusedImage& image, const ImageOutputMetadata& meta,
                      const std::string& filepath);

/**
 * @brief 写出 GeoTIFF sidecar: base.raw(原始 float32 交错) + base.json(元数据)。
 *
 * sidecar JSON 包含: 图像尺寸、斜距信息、像素间距、经纬度原点、snr_db、source。
 * is_placeholder 为 true 时同样安全跳过。
 *
 * @return 成功时返回 true。
 */
bool WriteGeoTiffSidecar(const ::sar::session::SarFocusedImage& image, const ImageOutputMetadata& meta,
                         const std::string& base_filepath);

// ── HDF5 输出（C++17 编译时启用，C++11 自动跳过）─────────────

#if defined(ONEQ_ENABLE_HDF5_OUTPUT)
/**
 * @brief 写出 HDF5 图像文件(dataset /image/real, /image/imag + 元数据 attrs)。
 *        需要 HighFive 库（C++17）。
 *
 *        is_placeholder 为 true 时安全跳过, 返回 true。
 */
bool WriteHdf5Image(const ::sar::session::SarFocusedImage& image,
                    const ImageOutputMetadata& meta,
                    const std::string& filepath);
#endif  // ONEQ_ENABLE_HDF5_OUTPUT

}  // namespace output
}  // namespace sar

#endif  // ONEQ_SRC_SAR_OUTPUT_IMAGE_FORMATTER_H_
