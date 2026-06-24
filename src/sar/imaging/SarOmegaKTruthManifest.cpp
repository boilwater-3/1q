#include "sar/imaging/SarOmegaKTruthManifest.h"

#include <cctype>
#include <cmath>
#include <sstream>

namespace sar {
namespace imaging {

namespace {

OmegaKTruthManifestParseResult Reject(OmegaKTruthManifestReason reason) {
  OmegaKTruthManifestParseResult result;
  result.reason = reason;
  return result;
}

bool ReadKey(std::istringstream* stream, const char* expected) {
  std::string key;
  return (*stream >> key) && key == expected;
}

bool ReadBool(std::istringstream* stream, bool* value) {
  std::string token;
  if (!(*stream >> token)) {
    return false;
  }
  if (token == "true") {
    *value = true;
    return true;
  }
  if (token == "false") {
    *value = false;
    return true;
  }
  return false;
}

bool IsFiniteNonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool IsSha256Hex(const std::string& value) {
  if (value.size() != 64U) {
    return false;
  }
  for (char character : value) {
    if (!std::isxdigit(static_cast<unsigned char>(character))) {
      return false;
    }
  }
  return true;
}

bool HasNoTrailingContent(std::istringstream* stream) {
  std::string trailing;
  return !(*stream >> trailing);
}

}  // namespace

OmegaKTruthManifestParseResult ParseOmegaKTruthManifest(const std::string& text) {
  std::istringstream stream(text);
  std::string magic;
  unsigned int envelope_version = 0U;
  if (!(stream >> magic >> envelope_version) ||
      magic != "ONEQ_SAR_OMEGA_K_TRUTH_MANIFEST") {
    return Reject(OmegaKTruthManifestReason::kInvalidHeader);
  }
  if (envelope_version != 1U) {
    return Reject(OmegaKTruthManifestReason::kUnsupportedVersion);
  }

  OmegaKTruthManifest manifest;
  if (!ReadKey(&stream, "dataset_id") || !(stream >> manifest.dataset_id) ||
      manifest.dataset_id.empty() ||
      !ReadKey(&stream, "schema_version") || !(stream >> manifest.schema_version)) {
    return Reject(OmegaKTruthManifestReason::kInvalidField);
  }
  if (manifest.schema_version != 1U) {
    return Reject(OmegaKTruthManifestReason::kUnsupportedVersion);
  }
  if (!ReadKey(&stream, "physical_evidence") ||
      !ReadBool(&stream, &manifest.physical_evidence) ||
      !ReadKey(&stream, "source") || !(stream >> manifest.source) ||
      !ReadKey(&stream, "acquisition_date") || !(stream >> manifest.acquisition_date) ||
      !ReadKey(&stream, "independently_generated") ||
      !ReadBool(&stream, &manifest.truth.independently_generated) ||
      !ReadKey(&stream, "inside_common_support") ||
      !ReadBool(&stream, &manifest.truth.inside_common_support) ||
      !ReadKey(&stream, "absolute_slant_range_m") ||
      !(stream >> manifest.truth.absolute_slant_range_m) ||
      !ReadKey(&stream, "azimuth_coordinate") ||
      !(stream >> manifest.truth.azimuth_coordinate) ||
      !ReadKey(&stream, "peak_phase_rad") ||
      !(stream >> manifest.truth.peak_phase_rad) ||
      !ReadKey(&stream, "peak_magnitude") ||
      !(stream >> manifest.truth.peak_magnitude) ||
      !ReadKey(&stream, "range_mainlobe_half_width") ||
      !(stream >> manifest.truth.range_mainlobe_half_width) ||
      !ReadKey(&stream, "azimuth_mainlobe_half_width") ||
      !(stream >> manifest.truth.azimuth_mainlobe_half_width) ||
      !ReadKey(&stream, "maximum_range_error_m") ||
      !(stream >> manifest.tolerances.maximum_range_error_m) ||
      !ReadKey(&stream, "maximum_azimuth_error") ||
      !(stream >> manifest.tolerances.maximum_azimuth_error) ||
      !ReadKey(&stream, "maximum_abs_phase_error_rad") ||
      !(stream >> manifest.tolerances.maximum_abs_phase_error_rad) ||
      !ReadKey(&stream, "maximum_relative_magnitude_error") ||
      !(stream >> manifest.tolerances.maximum_relative_magnitude_error) ||
      !ReadKey(&stream, "maximum_range_pslr_db") ||
      !(stream >> manifest.tolerances.maximum_range_pslr_db) ||
      !ReadKey(&stream, "maximum_azimuth_pslr_db") ||
      !(stream >> manifest.tolerances.maximum_azimuth_pslr_db) ||
      !ReadKey(&stream, "maximum_range_islr_db") ||
      !(stream >> manifest.tolerances.maximum_range_islr_db) ||
      !ReadKey(&stream, "maximum_azimuth_islr_db") ||
      !(stream >> manifest.tolerances.maximum_azimuth_islr_db) ||
      !ReadKey(&stream, "digest_sha256") || !(stream >> manifest.digest_sha256)) {
    return Reject(OmegaKTruthManifestReason::kInvalidField);
  }

  if (!std::isfinite(manifest.truth.absolute_slant_range_m) ||
      manifest.truth.absolute_slant_range_m < 0.0 ||
      !std::isfinite(manifest.truth.azimuth_coordinate) ||
      !std::isfinite(manifest.truth.peak_phase_rad) ||
      !std::isfinite(manifest.truth.peak_magnitude) ||
      manifest.truth.peak_magnitude <= 0.0 ||
      !IsFiniteNonnegative(manifest.tolerances.maximum_range_error_m) ||
      !IsFiniteNonnegative(manifest.tolerances.maximum_azimuth_error) ||
      !IsFiniteNonnegative(manifest.tolerances.maximum_abs_phase_error_rad) ||
      !IsFiniteNonnegative(manifest.tolerances.maximum_relative_magnitude_error) ||
      !std::isfinite(manifest.tolerances.maximum_range_pslr_db) ||
      !std::isfinite(manifest.tolerances.maximum_azimuth_pslr_db) ||
      !std::isfinite(manifest.tolerances.maximum_range_islr_db) ||
      !std::isfinite(manifest.tolerances.maximum_azimuth_islr_db)) {
    return Reject(OmegaKTruthManifestReason::kInvalidField);
  }
  if (!IsSha256Hex(manifest.digest_sha256)) {
    return Reject(OmegaKTruthManifestReason::kInvalidDigest);
  }
  std::string end;
  if (!(stream >> end) || end != "END" || !HasNoTrailingContent(&stream)) {
    return Reject(OmegaKTruthManifestReason::kUnexpectedContent);
  }

  OmegaKTruthManifestParseResult result;
  result.status = OmegaKTruthManifestStatus::kParsed;
  result.reason = OmegaKTruthManifestReason::kNone;
  result.manifest = manifest;
  return result;
}

}  // namespace imaging
}  // namespace sar
