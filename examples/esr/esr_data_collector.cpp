// Copyright 2026. All Rights Reserved.
//
// @file esr_data_collector.cpp
// @brief ESR 数据采集示例：使用 EsrTraceSession 将 config/input/output 写入 JSONL。

#include <iostream>
#include <memory>
#include <string>

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/trace/TraceSink.h"

int main() {
  namespace esr = electronic_surveillance_radar;

  const std::string trace_path = "/tmp/1q-esr-data-collector.jsonl";
  std::shared_ptr<oneq::trace::TraceSink> sink(
      new oneq::trace::JsonlFileTraceSink(trace_path, false));

  const esr::session::EsrSessionConfig config =
      esr::config::EsrSessionConfigBuilder().WithDetectionMinSnrDb(0.0f).Build();
  esr::session::EsrTraceSession session(config, esr::session::EsrTraceSessionOptions{sink, true});

  esr::session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.platform_pose.position_m.z = 2500.0f;

  const esr::session::EsrCycleResult result = session.StepWithResult(input);
  std::cout << "trace_file=" << trace_path
            << " observations=" << result.output_frame.observation_output.observations.size()
            << " hypotheses=" << result.output_frame.emitter_output.hypotheses.size()
            << std::endl;
  return 0;
}

