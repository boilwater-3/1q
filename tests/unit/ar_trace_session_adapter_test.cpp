/**
 * @file trace_session_adapter_test.cpp
 * @brief 验证三模块 TraceSession 中间层能够落盘记录 config/input/output。
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarTraceSession.h"
#include "1q/common/trace/TraceSink.h"
#include "1q/electro_optical_sensor/model/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/tools/EsrTraceSession.h"

namespace {

std::string MakeTempTracePath(const char* prefix) {
  std::ostringstream stream;
  stream << "/tmp/" << prefix << "-" << std::time(nullptr) << "-" << std::rand() << ".jsonl";
  return stream.str();
}

std::string ReadFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

void ExpectCommonTracePhases(const std::string& content, const std::string& module_name) {
  EXPECT_NE(content.find("\"module\":\"" + module_name + "\""), std::string::npos);
  EXPECT_NE(content.find("\"phase\":\"config\""), std::string::npos);
  EXPECT_NE(content.find("\"phase\":\"input\""), std::string::npos);
  EXPECT_NE(content.find("\"phase\":\"output\""), std::string::npos);
}

}  // namespace

namespace airborne_radar {
namespace tests {

TEST(TraceSessionAdapterTest, RadarTraceSessionWritesConfigInputOutput) {
  const std::string trace_path = MakeTempTracePath("oneq-radar-trace");
  std::shared_ptr<oneq::common::trace::TraceSink> sink(
      new oneq::common::trace::JsonlFileTraceSink(trace_path, false));

  session::RadarSessionConfig config = config::MakeDefaultRadarSessionConfig();
  config.detection.min_detection_margin_db = -100.0f;

  session::RadarTraceSession session(config, session::RadarTraceSessionOptions{sink, true});
  session::RadarCycleInput input;
  input.dt_sec = 1.0f;

  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.track_output_frame.published_track_count, 0U);

  const std::string content = ReadFile(trace_path);
  ExpectCommonTracePhases(content, "airborne_radar");

  std::remove(trace_path.c_str());
}

}  // namespace tests
}  // namespace airborne_radar

namespace electronic_surveillance_radar {
namespace tests {

TEST(TraceSessionAdapterTest, EsrTraceSessionWritesConfigInputOutput) {
  const std::string trace_path = MakeTempTracePath("oneq-esr-trace");
  std::shared_ptr<oneq::common::trace::TraceSink> sink(
      new oneq::common::trace::JsonlFileTraceSink(trace_path, false));

  core::session::EsrSessionConfig config;
  config.pipeline_config.scan.az_step_deg = 120.0f;
  config.pipeline_config.scan.el_step_deg = 40.0f;

  tools::EsrTraceSession session(config, tools::EsrTraceSessionOptions{sink, true});

  core::context::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  const core::session::EsrCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.output_frame.observation_output.observations.size(), 0U);

  const std::string content = ReadFile(trace_path);
  ExpectCommonTracePhases(content, "electronic_surveillance_radar");

  std::remove(trace_path.c_str());
}

}  // namespace tests
}  // namespace electronic_surveillance_radar

namespace electro_optical_sensor {
namespace tests {

TEST(TraceSessionAdapterTest, EosTraceSessionWritesConfigInputOutput) {
  const std::string trace_path = MakeTempTracePath("oneq-eos-trace");
  std::shared_ptr<oneq::common::trace::TraceSink> sink(
      new oneq::common::trace::JsonlFileTraceSink(trace_path, false));

  core::session::EosSessionConfig config;
  config.minimum_snr_db = 0.0f;

  tools::EosTraceSession session(config, tools::EosTraceSessionOptions{sink, true});

  core::model::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  const core::session::EosCycleResult result = session.StepWithResult(input);
  EXPECT_GE(result.output_frame.detections.size(), 0U);

  const std::string content = ReadFile(trace_path);
  ExpectCommonTracePhases(content, "electro_optical_sensor");

  std::remove(trace_path.c_str());
}

}  // namespace tests
}  // namespace electro_optical_sensor
