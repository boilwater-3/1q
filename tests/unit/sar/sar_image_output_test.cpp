/**
 * @file sar_image_output_test.cpp
 * @brief Binary 图像写入与 GeoTIFF sidecar 的单元测试。
 */

#include "sar/output/ImageFormatter.h"
#include "support/oneq_test_temp_dir.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace sar {
namespace output {
namespace {

sar::session::SarFocusedImage MakeTestImage(std::uint32_t rows, std::uint32_t cols) {
  sar::session::SarFocusedImage img;
  img.row_count = rows;
  img.column_count = cols;
  const std::size_t n = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  img.real_values.resize(n);
  img.imaginary_values.resize(n);
  for (std::size_t i = 0U; i < n; ++i) {
    img.real_values[i] = static_cast<double>(i) * 0.1;
    img.imaginary_values[i] = -static_cast<double>(i) * 0.05;
  }
  img.is_placeholder = false;
  return img;
}

ImageOutputMetadata MakeMeta() {
  ImageOutputMetadata m;
  m.center_slant_range_m = 5000.0;
  m.estimated_snr_db = 15.0;
  m.range_sample_count = 128U;
  m.azimuth_pulse_count = 64U;
  m.source = "rda_test";
  m.origin_lat_deg = 34.05;
  m.origin_lon_deg = -118.25;
  m.range_pixel_spacing_m = 1.5;
  m.azimuth_pixel_spacing_m = 2.0;
  return m;
}

std::uint32_t ReadUint32Le(std::istream& in) {
  unsigned char b[4];
  in.read(reinterpret_cast<char*>(b), 4);
  return static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8U) |
         (static_cast<std::uint32_t>(b[2]) << 16U) | (static_cast<std::uint32_t>(b[3]) << 24U);
}

float ReadFloat32Le(std::istream& in) {
  unsigned char b[4];
  in.read(reinterpret_cast<char*>(b), 4);
  const std::uint32_t u = static_cast<std::uint32_t>(b[0]) |
                          (static_cast<std::uint32_t>(b[1]) << 8U) |
                          (static_cast<std::uint32_t>(b[2]) << 16U) |
                          (static_cast<std::uint32_t>(b[3]) << 24U);
  union {
    std::uint32_t u;
    float f;
  } conv;
  conv.u = u;
  return conv.f;
}

}  // namespace

// ── WriteBinaryImage ─────────────────────────────────────────

TEST(SarImageOutputTest, BinaryWritesMagicAndHeader) {
  const sar::session::SarFocusedImage img = MakeTestImage(2U, 3U);
  const std::string path = oneq_test::TempDir() + "1q_sar_test_binary_header.bin";
  ASSERT_TRUE(WriteBinaryImage(img, MakeMeta(), path));

  std::ifstream in(path, std::ios::binary);
  ASSERT_TRUE(in.is_open());

  char magic[7];
  in.read(magic, 7);
  EXPECT_EQ(magic[0], '1');
  EXPECT_EQ(magic[1], 'Q');
  EXPECT_EQ(magic[2], 'S');
  EXPECT_EQ(magic[3], 'A');
  EXPECT_EQ(magic[4], 'R');
  EXPECT_EQ(magic[5], '\x01');
  EXPECT_EQ(magic[6], '\x00');

  const std::uint32_t rows = ReadUint32Le(in);
  const std::uint32_t cols = ReadUint32Le(in);
  EXPECT_EQ(rows, 2U);
  EXPECT_EQ(cols, 3U);

  std::remove(path.c_str());
}

TEST(SarImageOutputTest, BinaryRoundTripDataValues) {
  const std::uint32_t rows = 4U;
  const std::uint32_t cols = 5U;
  const sar::session::SarFocusedImage img = MakeTestImage(rows, cols);
  const std::string path = oneq_test::TempDir() + "1q_sar_test_binary_roundtrip.bin";
  ASSERT_TRUE(WriteBinaryImage(img, MakeMeta(), path));

  std::ifstream in(path, std::ios::binary);
  ASSERT_TRUE(in.is_open());
  in.seekg(7 + 4 + 4);  // skip magic + rows + cols

  const std::size_t n = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);

  // Check real values
  for (std::size_t i = 0U; i < n; ++i) {
    const float val = ReadFloat32Le(in);
    EXPECT_NEAR(val, static_cast<float>(img.real_values[i]), 1.0e-6f);
  }
  // Check imag values
  for (std::size_t i = 0U; i < n; ++i) {
    const float val = ReadFloat32Le(in);
    EXPECT_NEAR(val, static_cast<float>(img.imaginary_values[i]), 1.0e-6f);
  }

  std::remove(path.c_str());
}

TEST(SarImageOutputTest, BinarySkipsPlaceholder) {
  sar::session::SarFocusedImage img = MakeTestImage(2U, 2U);
  img.is_placeholder = true;
  const std::string path = oneq_test::TempDir() + "1q_sar_test_binary_placeholder.bin";
  EXPECT_TRUE(WriteBinaryImage(img, MakeMeta(), path));

  // Placeholder 不应写盘
  std::ifstream in(path, std::ios::binary);
  EXPECT_FALSE(in.is_open());
}

TEST(SarImageOutputTest, BinarySkipsZeroSize) {
  sar::session::SarFocusedImage img;
  img.is_placeholder = false;
  img.row_count = 0U;
  img.column_count = 0U;
  const std::string path = oneq_test::TempDir() + "1q_sar_test_binary_zero.bin";
  EXPECT_TRUE(WriteBinaryImage(img, MakeMeta(), path));

  std::ifstream in(path, std::ios::binary);
  EXPECT_FALSE(in.is_open());
}

// ── WriteGeoTiffSidecar ──────────────────────────────────────

TEST(SarImageOutputTest, SidecarWritesRawAndJson) {
  const sar::session::SarFocusedImage img = MakeTestImage(3U, 4U);
  const std::string base = oneq_test::TempDir() + "1q_sar_test_sidecar";
  ASSERT_TRUE(WriteGeoTiffSidecar(img, MakeMeta(), base));

  // .raw 文件应存在且大小正确: 3*4*2*4 = 96 bytes
  {
    std::ifstream raw(base + ".raw", std::ios::binary);
    ASSERT_TRUE(raw.is_open());
    raw.seekg(0, std::ios::end);
    const std::streamsize sz = raw.tellg();
    EXPECT_EQ(static_cast<std::size_t>(sz), 3U * 4U * 2U * 4U);
  }

  // .json 文件应存在且包含关键字段
  {
    std::ifstream json(base + ".json");
    ASSERT_TRUE(json.is_open());
    std::string content((std::istreambuf_iterator<char>(json)),
                         std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("\"row_count\""), std::string::npos);
    EXPECT_NE(content.find("\"column_count\""), std::string::npos);
    EXPECT_NE(content.find("\"center_slant_range_m\""), std::string::npos);
    EXPECT_NE(content.find("\"source\""), std::string::npos);
    EXPECT_NE(content.find("rda_test"), std::string::npos);
  }

  std::remove((base + ".raw").c_str());
  std::remove((base + ".json").c_str());
}

TEST(SarImageOutputTest, SidecarSkipsPlaceholder) {
  sar::session::SarFocusedImage img = MakeTestImage(2U, 2U);
  img.is_placeholder = true;
  const std::string base = oneq_test::TempDir() + "1q_sar_test_sidecar_placeholder";
  EXPECT_TRUE(WriteGeoTiffSidecar(img, MakeMeta(), base));

  std::ifstream raw(base + ".raw", std::ios::binary);
  EXPECT_FALSE(raw.is_open());
  std::ifstream json(base + ".json");
  EXPECT_FALSE(json.is_open());
}

}  // namespace output
}  // namespace sar
