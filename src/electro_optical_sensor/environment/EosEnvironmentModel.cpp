#include "electro_optical_sensor/environment/EosEnvironmentModel.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace electro_optical_sensor {
namespace environment {

session::EosEnvironmentModelResult ResolveEnvironmentFactors(const session::EosEnvironmentModelInputs& inputs) {
  const float cloud_ratio = oneq::common::numerics::Clamp01(inputs.cloud_coverage_ratio);
  const float altitude_km = std::max(0.0f, std::fabs(inputs.platform_altitude_m) / 1000.0f);
  const float wind_speed_mps = std::max(0.0f, inputs.wind_speed_mps);
  const float base_aerosol = oneq::common::numerics::SafePositive(inputs.base_aerosol_density_factor, 1.0f);
  const float base_turbulence = oneq::common::numerics::SafePositive(inputs.base_turbulence_factor, 1.0f);

  session::EosEnvironmentModelResult result;
  result.aerosol_density_factor =
      base_aerosol * (1.0f + 0.25f * cloud_ratio + 0.02f * altitude_km);
  result.turbulence_factor = base_turbulence *
                             (1.0f + 0.03f * wind_speed_mps + 0.20f * cloud_ratio +
                              0.01f * altitude_km);
  result.path_radiance_scale_bias =
      1.0f + 0.1f * cloud_ratio + 0.005f * wind_speed_mps;

  if (inputs.atmospheric_physics.enable_physical_model) {
    const float humidity = oneq::common::numerics::Clamp01(
        inputs.atmospheric_physics.relative_humidity);
    result.aerosol_density_factor *= (1.0f + 0.3f * humidity);
    const float temp_deviation =
        (inputs.atmospheric_physics.temperature_k - 288.15f) / 50.0f;
    result.turbulence_factor *= (1.0f + 0.1f * std::fabs(temp_deviation));
  }

  result.aerosol_density_factor = oneq::common::numerics::SafePositive(result.aerosol_density_factor, 1.0f);
  result.turbulence_factor = oneq::common::numerics::SafePositive(result.turbulence_factor, 1.0f);
  result.path_radiance_scale_bias = oneq::common::numerics::SafePositive(result.path_radiance_scale_bias, 1.0f);
  return result;
}

}  // namespace environment
}  // namespace electro_optical_sensor
