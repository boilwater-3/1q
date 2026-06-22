#include "sar/output/ImageFormatter.h"

#include <fstream>
#include <sstream>

namespace sar {
namespace output {

namespace {

bool IsEmptyOrPlaceholder(const ::sar::session::SarFocusedImage& image) {
  return image.is_placeholder || image.row_count == 0U || image.column_count == 0U ||
         image.real_values.empty() || image.imaginary_values.empty() ||
         image.real_values.size() != image.imaginary_values.size();
}

void WriteUint32Le(std::ostream& out, std::uint32_t value) {
  const unsigned char bytes[4] = {static_cast<unsigned char>(value & 0xFFU),
                                   static_cast<unsigned char>((value >> 8U) & 0xFFU),
                                   static_cast<unsigned char>((value >> 16U) & 0xFFU),
                                   static_cast<unsigned char>((value >> 24U) & 0xFFU)};
  out.write(reinterpret_cast<const char*>(bytes), 4);
}

void WriteFloat32Le(std::ostream& out, float value) {
  union {
    float f;
    std::uint32_t u;
  } conv;
  conv.f = value;
  const unsigned char bytes[4] = {static_cast<unsigned char>(conv.u & 0xFFU),
                                   static_cast<unsigned char>((conv.u >> 8U) & 0xFFU),
                                   static_cast<unsigned char>((conv.u >> 16U) & 0xFFU),
                                   static_cast<unsigned char>((conv.u >> 24U) & 0xFFU)};
  out.write(reinterpret_cast<const char*>(bytes), 4);
}

void WriteJsonDouble(std::ostream& out, const char* key, double value) {
  out << "  \"" << key << "\": " << value << ",\n";
}

void WriteJsonUint(std::ostream& out, const char* key, std::uint32_t value) {
  out << "  \"" << key << "\": " << value << ",\n";
}

void WriteJsonString(std::ostream& out, const char* key, const std::string& value) {
  out << "  \"" << key << "\": \"";
  for (char c : value) {
    if (c == '"' || c == '\\') {
      out << '\\';
    }
    out << c;
  }
  out << "\",\n";
}

}  // namespace

bool WriteBinaryImage(const ::sar::session::SarFocusedImage& image,
                      const ImageOutputMetadata& /*meta*/,
                      const std::string& filepath) {
  if (IsEmptyOrPlaceholder(image)) {
    return true;
  }

  std::ofstream file(filepath, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    return false;
  }

  // Magic: "1QSAR\x01\x00" (6 bytes)
  const char magic[] = {'1', 'Q', 'S', 'A', 'R', '\x01', '\x00'};
  file.write(magic, 7);

  // Header: rows (u32 LE), cols (u32 LE)
  WriteUint32Le(file, image.row_count);
  WriteUint32Le(file, image.column_count);

  // Data: real[] + imag[] as float32 LE
  const std::size_t total = image.real_values.size();
  for (std::size_t i = 0U; i < total; ++i) {
    WriteFloat32Le(file, static_cast<float>(image.real_values[i]));
  }
  for (std::size_t i = 0U; i < total; ++i) {
    WriteFloat32Le(file, static_cast<float>(image.imaginary_values[i]));
  }

  file.close();
  return !file.fail();
}

bool WriteGeoTiffSidecar(const ::sar::session::SarFocusedImage& image,
                         const ImageOutputMetadata& meta,
                         const std::string& base_filepath) {
  if (IsEmptyOrPlaceholder(image)) {
    return true;
  }

  // 1) Write .raw file: float32 interleaved (real, imag, real, imag, ...)
  const std::string raw_path = base_filepath + ".raw";
  {
    std::ofstream raw(raw_path, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!raw.is_open()) {
      return false;
    }
    const std::size_t total = image.real_values.size();
    for (std::size_t i = 0U; i < total; ++i) {
      WriteFloat32Le(raw, static_cast<float>(image.real_values[i]));
      WriteFloat32Le(raw, static_cast<float>(image.imaginary_values[i]));
    }
    raw.close();
    if (raw.fail()) {
      return false;
    }
  }

  // 2) Write .json sidecar
  const std::string json_path = base_filepath + ".json";
  {
    std::ofstream json(json_path, std::ios::out | std::ios::trunc);
    if (!json.is_open()) {
      return false;
    }

    json << "{\n";
    WriteJsonUint(json, "row_count", image.row_count);
    WriteJsonUint(json, "column_count", image.column_count);
    WriteJsonDouble(json, "center_slant_range_m", meta.center_slant_range_m);
    WriteJsonDouble(json, "estimated_snr_db", meta.estimated_snr_db);
    WriteJsonDouble(json, "range_pixel_spacing_m", meta.range_pixel_spacing_m);
    WriteJsonDouble(json, "azimuth_pixel_spacing_m", meta.azimuth_pixel_spacing_m);
    WriteJsonDouble(json, "origin_lat_deg", meta.origin_lat_deg);
    WriteJsonDouble(json, "origin_lon_deg", meta.origin_lon_deg);
    WriteJsonString(json, "source", meta.source);
    WriteJsonString(json, "format", "float32_interleaved_real_imag");
    json << "  \"raw_file\": \"" << (base_filepath + ".raw") << "\"\n";
    json << "}\n";

    json.close();
    if (json.fail()) {
      return false;
    }
  }

  return true;
}

// ── HDF5 输出(条件编译) ──────────────────────────────────────

#if defined(ONEQ_ENABLE_HDF5_OUTPUT)
#include <highfive/H5File.hpp>

bool WriteHdf5Image(const ::sar::session::SarFocusedImage& image,
                    const ImageOutputMetadata& meta,
                    const std::string& filepath) {
  if (IsEmptyOrPlaceholder(image)) {
    return true;
  }

  try {
    HighFive::File file(filepath, HighFive::File::Overwrite);

    const std::size_t rows = static_cast<std::size_t>(image.row_count);
    const std::size_t cols = static_cast<std::size_t>(image.column_count);

    // Dataset /image/real
    std::vector<std::vector<double>> real_2d(rows, std::vector<double>(cols));
    // Dataset /image/imag
    std::vector<std::vector<double>> imag_2d(rows, std::vector<double>(cols));

    for (std::size_t r = 0U; r < rows; ++r) {
      for (std::size_t c = 0U; c < cols; ++c) {
        const std::size_t idx = r * cols + c;
        real_2d[r][c] = image.real_values[idx];
        imag_2d[r][c] = image.imaginary_values[idx];
      }
    }

    HighFive::DataSet ds_real = file.createDataSet<double>("/image/real",
        HighFive::DataSpace::From(real_2d));
    ds_real.write(real_2d);

    HighFive::DataSet ds_imag = file.createDataSet<double>("/image/imag",
        HighFive::DataSpace::From(imag_2d));
    ds_imag.write(imag_2d);

    // Metadata as attributes
    auto root = file.getGroup("/");
    root.createAttribute<double>("center_slant_range_m", HighFive::DataSpace::From(meta.center_slant_range_m))
        .write(meta.center_slant_range_m);
    root.createAttribute<double>("estimated_snr_db", HighFive::DataSpace::From(meta.estimated_snr_db))
        .write(meta.estimated_snr_db);
    root.createAttribute<std::string>("source", HighFive::DataSpace::From(meta.source))
        .write(meta.source);

    return true;
  } catch (const std::exception&) {
    return false;
  }
}
#endif  // ONEQ_ENABLE_HDF5_OUTPUT

}  // namespace output
}  // namespace sar
