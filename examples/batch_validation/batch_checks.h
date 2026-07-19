/**
 * @file batch_checks.h
 * @brief 专项序列的结构化契约检查记录器。
 */
#ifndef EXAMPLES_BATCH_VALIDATION_BATCH_CHECKS_H_
#define EXAMPLES_BATCH_VALIDATION_BATCH_CHECKS_H_

#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

#include "batch_assertions.h"
#include "batch_csv_writer.h"

namespace batch_validation {

struct ContractCheck {
  std::string scenario_id;
  std::string phase;
  std::uint32_t cycle_index{0U};
  std::string check_id;
  std::string expected;
  std::string actual;
  bool passed{false};
  Severity severity{Severity::kError};
};

class ContractCheckCollector {
 public:
  void Add(const std::string& scenario_id, const std::string& phase,
           std::uint32_t cycle_index, const std::string& check_id,
           const std::string& expected, const std::string& actual, bool passed,
           Severity severity = Severity::kError) {
    checks_.push_back(
        {scenario_id, phase, cycle_index, check_id, expected, actual, passed, severity});
    std::fprintf(stderr, "    [check %s] %s phase=%s cycle=%u expected=%s actual=%s\n",
                 passed ? "PASS" : "FAIL", check_id.c_str(), phase.c_str(), cycle_index,
                 expected.c_str(), actual.c_str());
  }

  std::size_t size() const { return checks_.size(); }
  std::size_t FailureCount(Severity minimum = Severity::kError) const {
    std::size_t count = 0U;
    for (const ContractCheck& check : checks_) {
      if (!check.passed && static_cast<int>(check.severity) >= static_cast<int>(minimum)) ++count;
    }
    return count;
  }
  const std::vector<ContractCheck>& checks() const { return checks_; }

  bool WriteCsv(const std::string& path) const {
    CsvWriter writer(path,
                     "scenario_id,phase,cycle_index,check_id,expected,actual,passed,severity");
    for (const ContractCheck& check : checks_) {
      std::fprintf(writer.file(), "%s,%s,%u,%s,%s,%s,%d,%s\n",
                   EscapeCsvField(check.scenario_id).c_str(),
                   EscapeCsvField(check.phase).c_str(), check.cycle_index,
                   EscapeCsvField(check.check_id).c_str(),
                   EscapeCsvField(check.expected).c_str(),
                   EscapeCsvField(check.actual).c_str(), static_cast<int>(check.passed),
                   check.severity == Severity::kError ? "error" :
                   check.severity == Severity::kWarning ? "warning" : "info");
    }
    writer.Flush();
    return true;
  }

 private:
  std::vector<ContractCheck> checks_;
};

}  // namespace batch_validation
#endif  // EXAMPLES_BATCH_VALIDATION_BATCH_CHECKS_H_
