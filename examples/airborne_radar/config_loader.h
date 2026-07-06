#ifndef EXAMPLES_AR_CONFIG_LOADER_H_
#define EXAMPLES_AR_CONFIG_LOADER_H_

#include <string>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "config_loader_common.h"
#include "config_loader_detail.h"
#include "json_reader.h"

namespace examples {

/// Load an airborne_radar::config::ArSessionConfig from a parsed JSON object.
inline void LoadArSessionConfig(
    const examples::JsonValue& root,
    airborne_radar::config::ArSessionConfig* config) {
  LoadHardware(root["hardware"], &config->hardware);
  LoadMission(root["mission"], &config->mission);
  LoadPolicy(root["policy"], &config->policy);
  LoadEnvironment(root["environment"], &config->environment);
  config->environment.jamming_sensitivity_profile =
      JammingSensFromString(root["jamming_sensitivity_profile"].AsString());
}

/// Load an airborne_radar::config::ArSessionConfig from a JSON file.
inline bool LoadArSessionConfigFromFile(
    const char* path, airborne_radar::config::ArSessionConfig* config,
    std::string* error_msg) {
  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != examples::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadArSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_AR_CONFIG_LOADER_H_
