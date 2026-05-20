#include "electro_optical_sensor/environment/EosEnvironmentModel.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace electro_optical_sensor {
namespace environment {

namespace {


}  // namespace

EosEnvironmentModelResult ResolveEnvironmentFactors(const EosEnvironmentModelInputs& inputs) {
  const float cloud_ratio = oneq::internal::numerics::Clamp01(inputs.cloud_coverage_ratio);
  const float altitude_km = std::max(0.0f, std::fabs(inputs.platform_altitude_m) / 1000.0f);
  const float wind_speed_mps = std::max(0.0f, inputs.wind_speed_mps);
  const float base_aerosol = 1.0f;
  const float base_turbulence = 1.0f;

  EosEnvironmentModelResult result;
  if (inputs.model_type == EosEnvironmentModelType::kAdvanced) {
    result.aerosol_density_factor =
        base_aerosol * (1.0f + 0.25f * cloud_ratio + 0.02f * altitude_km);
    result.turbulence_factor = base_turbulence *
                               (1.0f + 0.03f * wind_speed_mps + 0.20f * cloud_ratio +
                                0.01f * altitude_km);
    result.path_radiance_scale_bias =
        1.0f + 0.1f * cloud_ratio + 0.005f * wind_speed_mps;
  } else {
    result.aerosol_density_factor = base_aerosol;
    result.turbulence_factor = base_turbulence;
    result.path_radiance_scale_bias = 1.0f;
  }

  result.aerosol_density_factor = oneq::internal::numerics::SafePositive(result.aerosol_density_factor, 1.0f);
  result.turbulence_factor = oneq::internal::numerics::SafePositive(result.turbulence_factor, 1.0f);
  result.path_radiance_scale_bias = oneq::internal::numerics::SafePositive(result.path_radiance_scale_bias, 1.0f);
  return result;
}

}  // namespace environment
}  // namespace electro_optical_sensor
