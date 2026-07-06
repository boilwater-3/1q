#ifndef EXAMPLES_EOS_CONFIG_LOADER_H_
#define EXAMPLES_EOS_CONFIG_LOADER_H_

#include <string>

#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "json_reader.h"
#include "config_loader_common.h"
#include "config_loader_detail.h"

namespace examples {

/// Load an config::EosSessionConfig from a parsed JSON object.
inline void LoadEosSessionConfig(
    const examples::JsonValue& root,
    electro_optical_sensor::config::EosSessionConfig* config) {
  LoadEosHardware(root["hardware"], &config->hardware);
  LoadEosMission(root["mission"], &config->mission);
  LoadEosPolicy(root["policy"], &config->policy);
  LoadEosEnvironment(root["environment"], &config->environment);
}

/// Load an config::EosSessionConfig from a JSON file.
inline bool LoadEosSessionConfigFromFile(
    const char* path, electro_optical_sensor::config::EosSessionConfig* config,
    std::string* error_msg) {
  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != examples::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadEosSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_EOS_CONFIG_LOADER_H_
