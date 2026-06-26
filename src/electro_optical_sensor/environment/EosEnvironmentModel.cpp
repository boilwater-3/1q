#include "electro_optical_sensor/environment/EosEnvironmentModel.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace electro_optical_sensor {
namespace environment {

namespace {


}  // namespace

session::EosEnvironmentModelResult ResolveEnvironmentFactors(const session::EosEnvironmentModelInputs& inputs) {
  const float cloud_ratio = oneq::internal::numerics::Clamp01(inputs.cloud_coverage_ratio);
  const float altitude_km = std::max(0.0f, std::fabs(inputs.platform_altitude_m) / 1000.0f);
  const float wind_speed_mps = std::max(0.0f, inputs.wind_speed_mps);
  const float base_aerosol = oneq::internal::numerics::SafePositive(inputs.base_aerosol_density_factor, 1.0f);
  const float base_turbulence = oneq::internal::numerics::SafePositive(inputs.base_turbulence_factor, 1.0f);

  session::EosEnvironmentModelResult result;
  if (inputs.model_type == config::EosEnvironmentModelType::kAdvanced) {
    result.aerosol_density_factor =
        base_aerosol * (1.0f + 0.25f * cloud_ratio + 0.02f * altitude_km);
    result.turbulence_factor = base_turbulence *
                               (1.0f + 0.03f * wind_speed_mps + 0.20f * cloud_ratio +
                                0.01f * altitude_km);
    result.path_radiance_scale_bias =
        1.0f + 0.1f * cloud_ratio + 0.005f * wind_speed_mps;

    // 若提供大气物理观测，用湿度修正气溶胶因子、温度修正湍流因子
    if (inputs.has_atmospheric_observation &&
        inputs.atmospheric_observation.enable_physical_model) {
      const float humidity = oneq::internal::numerics::Clamp01(
          inputs.atmospheric_observation.relative_humidity);
      result.aerosol_density_factor *= (1.0f + 0.3f * humidity);
      const float temp_deviation =
          (inputs.atmospheric_observation.temperature_k - 288.15f) / 50.0f;
      result.turbulence_factor *= (1.0f + 0.1f * std::fabs(temp_deviation));
    }
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
