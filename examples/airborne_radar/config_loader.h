#ifndef EXAMPLES_AR_CONFIG_LOADER_H_
#define EXAMPLES_AR_CONFIG_LOADER_H_

#include <string>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/foundation/json_reader.h"
#include "config_loader_common.h"
#include "config_loader_detail.h"

namespace examples {

/// Load an airborne_radar::session::RadarSessionConfig from a parsed JSON object.
inline void LoadArSessionConfig(
    const oneq::JsonValue& root,
    airborne_radar::session::RadarSessionConfig* config) {
  LoadHardware(root["hardware"], &config->hardware);
  LoadMission(root["mission"], &config->mission);
  LoadPolicy(root["policy"], &config->policy);
  LoadEnvironment(root["environment"], &config->environment);
  config->jamming_sensitivity_profile =
      JammingSensFromString(root["jamming_sensitivity_profile"].AsString());
}

/// Load an airborne_radar::session::RadarSessionConfig from a JSON file.
inline bool LoadArSessionConfigFromFile(
    const char* path, airborne_radar::session::RadarSessionConfig* config,
    std::string* error_msg) {
  oneq::JsonValue root;
  if (!oneq::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != oneq::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadArSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_AR_CONFIG_LOADER_H_
