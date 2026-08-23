/**
 * @file batch_replay.h
 * @brief 批量场景验证共享工具：可回放 trace 录制工厂 + 回放结果断言。
 *
 * @par 设计目标
 * 统一封装 ReplayTraceWriter 的构造（manifest 必填字段填充），
 * 让各传感器模块的批量程序用同一行调用为每个场景产出一个可回放 trace 目录，
 * 事后由各模块的 ReplayXxxTrace(trace_dir) 做确定性回归（分叉检测）。
 *
 * @par 关键约定
 * - 回放要求传 ReplayTraceWriter。库内周期落盘只有 Replay 目录。
 * - manifest.module 必须精确等于模块字符串常量，否则回放兼容性检查失败。
 * - 回放前必须 replay_writer->Flush()，且建议 writer 先析构（文件句柄释放）。
 */

#ifndef EXAMPLES_BATCH_VALIDATION_BATCH_REPLAY_H_
#define EXAMPLES_BATCH_VALIDATION_BATCH_REPLAY_H_

#include <cstdio>
#include <memory>
#include <string>

#include "1q/replay/ReplayTrace.h"

namespace batch_validation {

/// 模块名常量（必须与各 ReplayXxxTrace 内部期望的 module 字符串精确匹配）。
struct ModuleName {
  static constexpr const char* kAirborneRadar = "airborne_radar";
  static constexpr const char* kElectroOpticalSensor = "electro_optical_sensor";
  static constexpr const char* kElectronicSurveillanceRadar = "electronic_surveillance_radar";
  static constexpr const char* kSar = "sar";
  static constexpr const char* kSbirsSensor = "sbirs_sensor";
};

/**
 * @brief 为单个场景创建一个 ReplayTraceWriter。
 *
 * @param[in] trace_dir     trace 输出目录（每个场景应唯一）。
 * @param[in] module_name   模块字符串（取 ModuleName::kXxx 常量）。
 * @param[in] trace_id      本次批量运行的唯一 trace 标识（如 "ar-batch-20260702"）。
 * @param[in] scenario_id   本场景标识（如 "ar_r15km_rcs5_n3"）。
 * @return 共享持有的 ReplayTraceWriter（传给 XxxRecordingSessionOptions::replay_writer）。
 *
 * @note overwrite=true 会清空重建目录，避免残留事件干扰。其余 manifest 字段
 *       用默认值（schema_version=1, serializer_version 已默认 flatbuffers-v1）。
 */
inline std::shared_ptr<oneq::replay::ReplayTraceWriter> MakeReplayWriter(
    const std::string& trace_dir, const std::string& module_name,
    const std::string& trace_id, const std::string& scenario_id) {
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = trace_id;
  manifest.module = module_name;
  manifest.scenario_id = scenario_id;
  // overwrite=true：清空旧目录，保证本场景 trace 干净。
  return std::make_shared<oneq::replay::ReplayTraceWriter>(trace_dir, manifest, true);
}

/**
 * @brief 回放校验结果（模块无关的摘要）。
 *
 * 各模块的 ReplayXxxTrace 返回各自 XxxReplaySessionResult，结构形状一致；
 * 调用方把关键字段填入本结构，供共享的 CSV 输出与告警逻辑使用。
 */
struct ReplayCheckResult {
  bool ok{false};                         ///< 整体回放是否成功
  bool divergence_found{false};           ///< 是否检测到输出分叉
  std::uint64_t compared_output_count{0}; ///< 比对过的 cycle_output 数
  std::uint64_t applied_input_count{0};   ///< 应用过的 cycle_input 数
  bool reached_failure_marker{false};     ///< 是否因 failure_marker 停止
  std::string first_error;                ///< 首个错误描述（空表示无错误）
};

/**
 * @brief 打印回放结果摘要到 stderr（供运行时观察）。
 *
 * @param[in] scenario_id  场景标识（仅用于日志前缀）。
 * @param[in] r             回放校验结果。
 * @param[in] expected_outputs  本场景期望比对的 cycle_output 数（== Step 次数）。
 */
inline void LogReplayResult(const std::string& scenario_id, const ReplayCheckResult& r,
                            std::uint64_t expected_outputs) {
  if (r.ok && !r.divergence_found && r.compared_output_count == expected_outputs) {
    std::fprintf(stderr, "    [replay] %s: OK (compared=%llu)\n", scenario_id.c_str(),
                 static_cast<unsigned long long>(r.compared_output_count));
  } else {
    std::fprintf(stderr, "    [replay] %s: FAIL ok=%d divergence=%d compared=%llu/%llu err=%s\n",
                 scenario_id.c_str(), static_cast<int>(r.ok),
                 static_cast<int>(r.divergence_found),
                 static_cast<unsigned long long>(r.compared_output_count),
                 static_cast<unsigned long long>(expected_outputs), r.first_error.c_str());
  }
}

}  // namespace batch_validation

#endif  // EXAMPLES_BATCH_VALIDATION_BATCH_REPLAY_H_
