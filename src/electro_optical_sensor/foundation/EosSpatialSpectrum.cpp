#include "electro_optical_sensor/foundation/EosSpatialSpectrum.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace electro_optical_sensor {
namespace foundation {
namespace spatial_spectrum {

namespace {


}  // namespace

SpatialSpectrumResult EvaluateSpatialResolvability(const SpatialSpectrumInputs& inputs) {
  const float target_size_m = oneq::common::numerics::SafePositive(inputs.target_characteristic_size_m, 1.0f);
  const float gsd_m = oneq::common::numerics::SafePositive(inputs.ground_sample_distance_m, 0.2f);
  const float mtf_reference = oneq::common::numerics::Clamp01(inputs.optical_mtf_reference);
  const float sampling_efficiency =
      oneq::common::numerics::Clamp01(inputs.sampling_efficiency);
  const float contrast_ratio = oneq::common::numerics::Clamp01(inputs.scene_contrast_ratio);

  SpatialSpectrumResult result;
  result.target_frequency_cpm = 1.0f / target_size_m;
  result.nyquist_frequency_cpm = 0.5f / gsd_m;

  const float frequency_ratio =
      result.target_frequency_cpm / std::max(result.nyquist_frequency_cpm, 1.0e-6f);
  result.optical_pass_ratio =
      oneq::common::numerics::Clamp(1.0f - 0.5f * frequency_ratio, 0.0f, 1.0f);

  const float base_score = result.optical_pass_ratio * mtf_reference * sampling_efficiency;
  result.resolvability_score =
      oneq::common::numerics::Clamp(base_score * (0.5f + 0.5f * contrast_ratio), 0.0f, 1.0f);
  result.spectrum_quality_gain = oneq::common::numerics::Clamp(
      0.05f + 0.95f * result.resolvability_score, 0.05f, 1.0f);
  return result;
}

}  // namespace spatial_spectrum
}  // namespace foundation
}  // namespace electro_optical_sensor
