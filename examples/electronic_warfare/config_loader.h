#ifndef EXAMPLES_ESR_CONFIG_LOADER_H_
#define EXAMPLES_ESR_CONFIG_LOADER_H_

#include <string>

#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/foundation/json_reader.h"
#include "config_loader_common.h"
#include "config_loader_detail.h"

namespace examples {

/// Load an config::EsrSessionConfig from a parsed JSON object.
inline void LoadEsrSessionConfig(
    const oneq::JsonValue& root,
    electronic_surveillance_radar::config::EsrSessionConfig* config) {
  LoadEsrHardware(root["hardware"], &config->hardware);
  LoadEsrMission(root["mission"], &config->mission);
  LoadEsrPolicy(root["policy"], &config->policy);
  LoadEsrEnvironment(root["environment"], &config->environment);
}

/// Load an config::EsrSessionConfig from a JSON file.
inline bool LoadEsrSessionConfigFromFile(
    const char* path, electronic_surveillance_radar::config::EsrSessionConfig* config,
    std::string* error_msg) {
  oneq::JsonValue root;
  if (!oneq::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != oneq::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadEsrSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_ESR_CONFIG_LOADER_H_
