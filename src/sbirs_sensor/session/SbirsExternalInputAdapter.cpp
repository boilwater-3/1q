#include "1q/sbirs_sensor/session/SbirsExternalInputAdapter.h"

namespace sbirs_sensor {
namespace session {

SbirsCycleInput MakeSbirsCycleInput(std::uint32_t cycle_index, float dt_sec,
                                    double utc_julian_day,
                                    const SbirsVector3M& satellite_position_ecef_m,
                                    const SbirsSceneTargetList& scene) {
  SbirsCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = dt_sec;
  input.utc_julian_day = utc_julian_day;
  // 存在性标志必须与数据一致（docs/common/contract.md 规则 2）：本工厂已接收
  // 卫星位置，必须同步置位，否则输入会被 kInvalidSatellitePosition 拒绝。
  input.has_satellite_position = true;
  input.satellite_position_ecef_m = satellite_position_ecef_m;
  input.scene = scene;
  return input;
}

}  // namespace session
}  // namespace sbirs_sensor
