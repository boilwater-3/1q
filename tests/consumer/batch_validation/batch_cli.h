/**
 * @file batch_cli.h
 * @brief 批量验证统一命令行解析。
 */
#ifndef EXAMPLES_BATCH_VALIDATION_BATCH_CLI_H_
#define EXAMPLES_BATCH_VALIDATION_BATCH_CLI_H_

#include <cstdio>
#include <string>

namespace batch_validation {

enum class ScenarioSuite { kSweep, kSequence, kAll };

struct BatchCliOptions {
  ScenarioSuite suite{ScenarioSuite::kAll};
  std::string output_dir{};
  std::string scenario_id{};
  bool list_scenarios{false};
};

inline const char* SuiteName(ScenarioSuite suite) {
  switch (suite) {
    case ScenarioSuite::kSweep: return "sweep";
    case ScenarioSuite::kSequence: return "sequence";
    case ScenarioSuite::kAll: return "all";
  }
  return "all";
}

inline bool IncludesSweep(ScenarioSuite suite) {
  return suite == ScenarioSuite::kSweep || suite == ScenarioSuite::kAll;
}

inline bool IncludesSequence(ScenarioSuite suite) {
  return suite == ScenarioSuite::kSequence || suite == ScenarioSuite::kAll;
}

inline bool ParseSuite(const std::string& text, ScenarioSuite* suite) {
  if (suite == nullptr) return false;
  if (text == "sweep") *suite = ScenarioSuite::kSweep;
  else if (text == "sequence") *suite = ScenarioSuite::kSequence;
  else if (text == "all") *suite = ScenarioSuite::kAll;
  else return false;
  return true;
}

inline bool ParseBatchCli(int argc, char** argv, const char* default_output_dir,
                          BatchCliOptions* options, std::string* error) {
  if (options == nullptr || default_output_dir == nullptr) return false;
  options->output_dir = default_output_dir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--list-scenarios") {
      options->list_scenarios = true;
      continue;
    }
    if (arg == "--suite" || arg == "--scenario" || arg == "--output-dir") {
      if (i + 1 >= argc) {
        if (error != nullptr) *error = arg + " requires a value";
        return false;
      }
      const std::string value = argv[++i];
      if (arg == "--suite") {
        if (!ParseSuite(value, &options->suite)) {
          if (error != nullptr) *error = "invalid suite: " + value;
          return false;
        }
      } else if (arg == "--scenario") {
        options->scenario_id = value;
      } else {
        options->output_dir = value;
      }
      continue;
    }
    if (error != nullptr) *error = "unknown argument: " + arg;
    return false;
  }
  return true;
}

inline void PrintBatchUsage(const char* program) {
  std::fprintf(stderr,
               "Usage: %s [--suite sweep|sequence|all] [--scenario ID] "
               "[--output-dir PATH] [--list-scenarios]\n",
               program == nullptr ? "batch_validation" : program);
}

}  // namespace batch_validation
#endif  // EXAMPLES_BATCH_VALIDATION_BATCH_CLI_H_
