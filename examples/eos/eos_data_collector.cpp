// Copyright 2026. All Rights Reserved.
//
// @file eos_data_collector.cpp
// @brief EOS 数据采集示例：使用 EosTraceSession 将 config/input/output 写入 JSONL。

#include <iostream>
#include <memory>
#include <string>

#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/trace/TraceSink.h"

int main() {
  namespace eos = electro_optical_sensor;

  const std::string trace_path = "/tmp/1q-eos-data-collector.jsonl";
  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::JsonlFileTraceSink(trace_path, false));

  const eos::session::EosSessionConfig config =
      eos::config::EosSessionConfigBuilder()
          .WithDetectionProfile(eos::config::EosDetectionProfile::kAggressive)
          .Build();
  eos::session::EosTraceSession session(config, eos::session::EosTraceSessionOptions{sink, true});

  eos::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.platform_pose.position_m.z = 1200.0f;

  const eos::session::EosCycleResult result = session.StepWithResult(input);
  std::cout << "trace_file=" << trace_path
            << " executed=" << (result.executed_this_cycle ? "true" : "false")
            << " detections=" << result.output_frame.detections.size() << std::endl;
  return 0;
}
