#ifndef EXAMPLES_SAR_CONFIG_LOADER_H_
#define EXAMPLES_SAR_CONFIG_LOADER_H_

#include <string>

#include "json_reader.h"
#include "1q/sar/sar.hpp"
#include "config_loader_common.h"
#include "config_loader_detail.h"

namespace examples {

/// Load a sar::config::SarSessionConfig from a parsed JSON object.
inline void LoadSarSessionConfig(
    const examples::JsonValue& root,
    sar::config::SarSessionConfig* config) {
  LoadSarHardware(root["hardware"], &config->hardware);
  LoadSarMission(root["mission"], &config->mission);
  LoadSarProcessing(root["processing"], &config->policy);
  LoadSarEnvironment(root["environment"], &config->environment);
}

/// Load a sar::config::SarSessionConfig from a JSON file.
inline bool LoadSarSessionConfigFromFile(
    const char* path, sar::config::SarSessionConfig* config,
    std::string* error_msg) {
  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != examples::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadSarSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_SAR_CONFIG_LOADER_H_
