#ifndef EXAMPLES_ESR_CONFIG_LOADER_H_
#define EXAMPLES_ESR_CONFIG_LOADER_H_

#include <string>

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "json_reader.h"
#include "config_loader_common.h"
#include "config_loader_detail.h"

namespace examples {

/// Load an config::EsrSessionConfig from a parsed JSON object.
inline void LoadEsrSessionConfig(
    const examples::JsonValue& root,
    electronic_surveillance_radar::config::EsrSessionConfig* config) {
  LoadEsrHardware(root["hardware"], &config->hardware);
  LoadEsrMission(root["mission"], &config->mission);
  LoadEsrPolicy(root["policy"], &config->policy);
  LoadEsrEnvironment(root["environment"], &config->environment);
  // 电源状态单源（COMMON-OQ-4 字段提升）：session 级 sensor_enabled
  config->sensor_enabled = root["sensor_enabled"].IsNull() ||
                           root["sensor_enabled"].AsBool();
}

/// Load an config::EsrSessionConfig from a JSON file.
inline bool LoadEsrSessionConfigFromFile(
    const char* path, electronic_surveillance_radar::config::EsrSessionConfig* config,
    std::string* error_msg) {
  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != examples::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadEsrSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_ESR_CONFIG_LOADER_H_
