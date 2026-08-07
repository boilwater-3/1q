/**
 * @file debug_view_json_test.cpp
 * @brief 锁定 4 模块 DebugView → JSON 序列化输出格式（规则 12 参考实现）。
 *
 * 覆盖要点：
 *   - AR/EOS/SAR/SBIRS 四个模块序列化器对完整手填视图的逐字 JSON 输出断言；
 *   - JSON 转义（引号/反斜杠/换行/控制字符）；
 *   - 三种常见落盘模式参考（只落非标称行 / 跨周期状态增量 / 降频落盘）与
 *     行级序列化（AR/EOS/SBIRS 序列化器内 *WriteNonNominal* / *WriteStatusDeltas* /
 *     *WriteDownsampledView / *StateToJson）；
 *   - 同时证明恢复的 AR/EOS 序列化器能针对当前 DebugView 结构编译。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12 的参考实现
 * （examples/common 的 *DebugViewToJson.h 序列化器 + debug_view_json.h 共享原语）。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <string>
#include <unordered_map>

#include "ArDebugViewToJson.h"
#include "EosDebugViewToJson.h"
#include "SarDebugViewToJson.h"
#include "SbirsDebugViewToJson.h"

namespace ar = airborne_radar;
namespace eos = electro_optical_sensor;
namespace sbirs = sbirs_sensor;

namespace {

// 手填一个含轨迹与诊断的 AR 调试视图。
ar::session::ArTrackOutputDebugView MakeArView() {
  ar::session::ArTrackOutputDebugView view;
  view.world_cycle_index = 7U;
  view.output_cycle_index = 3U;
  view.completed_this_cycle = true;
  view.receiver_impairment = ar::session::ArReceiverImpairment::kSaturated;

  ar::session::ArDebugTrackState track;
  track.external_target_id = 42U;
  track.target_name = "tgt-\"A\"";
  track.status = ar::session::ArDebugTrackStatus::kConfirmed;
  track.present_in_input = true;
  track.has_track = true;
  track.association_key = 9U;
  track.position_x = 1.5F;
  track.position_y = -2.5F;
  track.position_z = 0.0F;
  track.speed = 300.0F;
  track.rcs = 2.0F;
  track.hit_count = 4U;
  track.miss_count = 1U;
  track.target_type = "threat";
  view.tracks.push_back(track);

  ar::session::ArIssue issue;
  issue.severity = ar::session::ArIssueSeverity::kWarning;
  issue.phase = ar::session::ArIssuePhase::kExecution;
  issue.code = "ar.impairment";
  issue.message = "saturated";
  view.issues.push_back(issue);
  return view;
}

// 手填一个含目标与问题条目的 EOS 调试视图。
eos::session::EosOutputDebugView MakeEosView() {
  eos::session::EosOutputDebugView view;
  view.input_cycle_index = 1U;
  view.output_cycle_index = 1U;
  view.executed_this_cycle = true;
  view.abort_reason = eos::session::EosPipelineAbortReason::kNone;

  eos::session::EosDebugTargetState target;
  target.target_id = 42U;
  target.target_name = "consumer-target";
  target.status = eos::session::EosDebugTargetStatus::kDetected;
  target.present_in_input = true;
  target.has_raw_output_record = true;
  target.detected = true;
  target.range_m = 1500.0F;
  target.azimuth_deg = 45.0F;
  target.elevation_deg = 30.0F;
  target.fused_snr_db = 12.0F;
  view.targets.push_back(target);

  eos::session::EosIssue issue;
  issue.severity = eos::session::EosIssueSeverity::kError;
  issue.phase = eos::session::EosIssuePhase::kInputValidation;
  issue.code = "eos.verification";
  issue.message = "synthetic";
  issue.field = "dt_sec";
  view.issues.push_back(issue);
  return view;
}

// 手填一个含点目标与空诊断的 SAR 调试视图。
sar::session::SarProductDebugView MakeSarView() {
  sar::session::SarProductDebugView view;
  view.input_cycle_index = 2U;
  view.output_cycle_index = 2U;
  view.executed_this_cycle = true;
  view.completed_stage = sar::session::SarProcessingStage::kL3BpImage;
  view.has_raw_echo = true;
  view.has_range_compressed_echo = true;
  view.has_l1_image = true;
  view.has_l3_bp_image = true;
  view.has_focused_pixels = true;
  view.estimated_snr_db = 12.5;
  view.range_sample_count = 512U;
  view.azimuth_pulse_count = 256U;

  sar::session::SarDebugPointTarget point_target;
  point_target.target_id = 1U;
  point_target.target_name = "pt-a";
  point_target.radar_cross_section_dbsm = -10.0;
  view.point_targets.push_back(point_target);
  return view;
}

// 手填一个含 coasting 目标与空诊断的 SBIRS 调试视图。
sbirs::session::SbirsOutputDebugView MakeSbirsView() {
  sbirs::session::SbirsOutputDebugView view;
  view.input_cycle_index = 5U;
  view.output_cycle_index = 5U;
  view.executed_this_cycle = true;
  view.abort_reason = sbirs::session::SbirsPipelineAbortReason::kNone;

  sbirs::session::SbirsDebugTargetState target;
  target.target_id = 7U;
  target.target_name = "sat-7";
  target.status = sbirs::session::SbirsDebugTargetStatus::kCoasting;
  target.present_in_input = true;
  target.has_raw_output_record = true;
  target.detected = true;
  target.tracking_source = sbirs::attribution::SbirsTrackingSource::kEstimated;
  target.estimated_range_m = 1000.0F;
  target.has_estimation_nis = true;
  target.estimation_nis = 0.5F;
  target.estimation_nis_gate_exceeded = false;
  target.nfov_channel_id = 2;
  target.has_nfov_tracking_diagnostics = true;
  target.nfov_pointing_error_deg = 0.1F;
  target.nfov_geometry_gate_passed = true;
  target.nfov_snr_gate_passed = true;
  target.nfov_tracking_gate_failure_count = 3U;
  target.nfov_tracking_coasting = true;
  target.azimuth_deg = 45.0F;
  target.elevation_deg = 30.0F;
  target.infrared_snr_linear = 2.0F;
  target.observation_stage = sbirs::output::SbirsObservationStage::kNarrowFieldTrack;
  view.targets.push_back(target);
  return view;
}

// 构造含两个目标（id 10/11）、状态可指定的 EOS 调试视图（供增量/非标称模式测试）。
eos::session::EosOutputDebugView MakeEosViewWithStatuses(
    std::uint32_t cycle, eos::session::EosDebugTargetStatus first,
    eos::session::EosDebugTargetStatus second) {
  eos::session::EosOutputDebugView view;
  view.input_cycle_index = cycle;
  view.executed_this_cycle = true;

  eos::session::EosDebugTargetState a;
  a.target_id = 10U;
  a.target_name = "a";
  a.status = first;
  a.present_in_input = true;
  view.targets.push_back(a);

  eos::session::EosDebugTargetState b;
  b.target_id = 11U;
  b.target_name = "b";
  b.status = second;
  b.present_in_input = true;
  view.targets.push_back(b);
  return view;
}

// 构造含两个目标（external_target_id 10/11）、状态可指定的 AR 调试视图。
ar::session::ArTrackOutputDebugView MakeArViewWithStatuses(
    std::uint64_t cycle, ar::session::ArDebugTrackStatus first,
    ar::session::ArDebugTrackStatus second) {
  ar::session::ArTrackOutputDebugView view;
  view.world_cycle_index = cycle;
  view.completed_this_cycle = true;

  ar::session::ArDebugTrackState a;
  a.external_target_id = 10U;
  a.target_name = "a";
  a.status = first;
  a.present_in_input = true;
  view.tracks.push_back(a);

  ar::session::ArDebugTrackState b;
  b.external_target_id = 11U;
  b.target_name = "b";
  b.status = second;
  b.present_in_input = true;
  view.tracks.push_back(b);
  return view;
}

}  // namespace

TEST(DebugViewJsonTest, ArDebugViewToJsonMatchesExpectedJson) {
  const std::string json = ArDebugViewToJson(MakeArView());
  EXPECT_EQ(json, R"({"world_cycle_index":7,"output_cycle_index":3,"completed_this_cycle":true,)"
                  R"("receiver_impairment":"saturated","tracks":[{"external_target_id":42,)"
                  R"("target_name":"tgt-\"A\"","status":"confirmed","present_in_input":true,)"
                  R"("has_track":true,"association_key":9,"position_x":1.5,"position_y":-2.5,)"
                  R"("position_z":0,"speed":300,"rcs":2,"hit_count":4,"miss_count":1,)"
                  R"("target_type":"threat"}],"issues":[{"severity":"warning",)"
                  R"("phase":1,"code":"ar.impairment","message":"saturated"}]})");
}

TEST(DebugViewJsonTest, EosDebugViewToJsonMatchesExpectedJson) {
  const std::string json = EosDebugViewToJson(MakeEosView());
  EXPECT_EQ(json,
            R"({"input_cycle_index":1,"output_cycle_index":1,"executed_this_cycle":true,)"
            R"("abort_reason":"none","targets":[{"target_id":42,)"
            R"("target_name":"consumer-target","status":"detected","present_in_input":true,)"
            R"("has_raw_output_record":true,"detected":true,"range_m":1500,"azimuth_deg":45,)"
            R"("elevation_deg":30,"fused_snr_db":12}],"issues":[{"severity":"error",)"
            R"("phase":0,"code":"eos.verification","message":"synthetic","field":"dt_sec"}]})");
}

TEST(DebugViewJsonTest, SarDebugViewToJsonMatchesExpectedJson) {
  const std::string json = SarDebugViewToJson(MakeSarView());
  EXPECT_EQ(json, R"({"input_cycle_index":2,"output_cycle_index":2,"executed_this_cycle":true,)"
                  R"("abort_reason":"","completed_stage":"l3_bp_image",)"
                  R"("has_raw_echo":true,"has_range_compressed_echo":true,"has_l1_image":true,)"
                  R"("has_l3_bp_image":true,"has_focused_pixels":true,"estimated_snr_db":12.5,)"
                  R"("range_sample_count":512,"azimuth_pulse_count":256,"point_targets":)"
                  R"([{"target_id":1,"target_name":"pt-a","radar_cross_section_dbsm":-10}],)"
                  R"("issues":[]})");
}

TEST(DebugViewJsonTest, SbirsDebugViewToJsonMatchesExpectedJson) {
  const std::string json = SbirsDebugViewToJson(MakeSbirsView());
  EXPECT_EQ(json, R"({"input_cycle_index":5,"output_cycle_index":5,"executed_this_cycle":true,)"
                  R"("abort_reason":"none","targets":)"
                  R"([{"target_id":7,"target_name":"sat-7","status":"coasting",)"
                  R"("present_in_input":true,"has_raw_output_record":true,"detected":true,)"
                  R"("tracking_source":"estimated","estimated_range_m":1000,)"
                  R"("has_estimation_nis":true,"estimation_nis":0.5,)"
                  R"("estimation_nis_gate_exceeded":false,"nfov_channel_id":2,)"
                  R"("has_nfov_tracking_diagnostics":true,"nfov_pointing_error_deg":0.1,)"
                  R"("nfov_geometry_gate_passed":true,"nfov_snr_gate_passed":true,)"
                  R"("nfov_tracking_gate_failure_count":3,"nfov_tracking_coasting":true,)"
                  R"("azimuth_deg":45,"elevation_deg":30,"infrared_snr_linear":2,)"
                  R"("observation_stage":"narrow_field_track"}],"issues":[]})");
}

TEST(DebugViewJsonTest, JsonEscapeHandlesQuotesBackslashesAndControlChars) {
  sar::session::SarProductDebugView view;
  // 依次含：双引号、反斜杠、换行、\x01 控制字符、字母 e。
  view.abort_reason = std::string("a\"b\\c\nd\x01") + "e";
  const std::string json = SarDebugViewToJson(view);
  EXPECT_NE(json.find(R"("abort_reason":"a\"b\\c\nd\u0001e")"), std::string::npos);
}

TEST(DebugViewJsonTest, EosDebugTargetStateToJsonMatchesFullViewRow) {
  const eos::session::EosOutputDebugView view = MakeEosView();
  EXPECT_EQ(EosDebugTargetStateToJson(view.targets[0]),
            R"({"target_id":42,"target_name":"consumer-target","status":"detected",)"
            R"("present_in_input":true,"has_raw_output_record":true,"detected":true,)"
            R"("range_m":1500,"azimuth_deg":45,"elevation_deg":30,"fused_snr_db":12})");
}

TEST(DebugViewJsonTest, EosWriteNonNominalTargetsSkipsDetectedRows) {
  // 目标 10 为标称（kDetected）被跳过，目标 11 为低于门限被写入。
  const eos::session::EosOutputDebugView view =
      MakeEosViewWithStatuses(1U, eos::session::EosDebugTargetStatus::kDetected,
                              eos::session::EosDebugTargetStatus::kObservedBelowThreshold);
  std::ostringstream out;
  EosWriteNonNominalTargets(view, out);
  EXPECT_EQ(out.str(),
            R"({"target_id":11,"target_name":"b","status":"observed_below_threshold",)"
            R"("present_in_input":true,"has_raw_output_record":false,"detected":false,)"
            R"("range_m":0,"azimuth_deg":0,"elevation_deg":0,"fused_snr_db":0})"
            "\n");
}

TEST(DebugViewJsonTest, EosWriteNonNominalTargetsEmptyWhenAllNominal) {
  const eos::session::EosOutputDebugView view =
      MakeEosViewWithStatuses(1U, eos::session::EosDebugTargetStatus::kDetected,
                              eos::session::EosDebugTargetStatus::kDetected);
  std::ostringstream out;
  EosWriteNonNominalTargets(view, out);
  EXPECT_TRUE(out.str().empty());
}

TEST(DebugViewJsonTest, EosWriteTargetStatusDeltasWritesOnlyChangedTargets) {
  std::unordered_map<std::uint64_t, eos::session::EosDebugTargetStatus> prev_status;
  std::ostringstream out;

  // 周期 1：两个目标均首次出现 → 两行都写。
  EosWriteTargetStatusDeltas(
      MakeEosViewWithStatuses(1U, eos::session::EosDebugTargetStatus::kDetected,
                              eos::session::EosDebugTargetStatus::kObservedBelowThreshold),
      prev_status, out);
  EXPECT_EQ(out.str(),
            R"({"target_id":10,"target_name":"a","status":"detected",)"
            R"("present_in_input":true,"has_raw_output_record":false,"detected":false,)"
            R"("range_m":0,"azimuth_deg":0,"elevation_deg":0,"fused_snr_db":0})"
            "\n"
            R"({"target_id":11,"target_name":"b","status":"observed_below_threshold",)"
            R"("present_in_input":true,"has_raw_output_record":false,"detected":false,)"
            R"("range_m":0,"azimuth_deg":0,"elevation_deg":0,"fused_snr_db":0})"
            "\n");

  // 周期 2：目标 10 状态未变（不写），目标 11 变为 kNotInOutput（写一行）。
  out.str("");
  EosWriteTargetStatusDeltas(
      MakeEosViewWithStatuses(2U, eos::session::EosDebugTargetStatus::kDetected,
                              eos::session::EosDebugTargetStatus::kNotInOutput),
      prev_status, out);
  EXPECT_EQ(out.str(),
            R"({"target_id":11,"target_name":"b","status":"not_in_output",)"
            R"("present_in_input":true,"has_raw_output_record":false,"detected":false,)"
            R"("range_m":0,"azimuth_deg":0,"elevation_deg":0,"fused_snr_db":0})"
            "\n");
  EXPECT_EQ(prev_status.at(10U), eos::session::EosDebugTargetStatus::kDetected);
  EXPECT_EQ(prev_status.at(11U), eos::session::EosDebugTargetStatus::kNotInOutput);
}

TEST(DebugViewJsonTest, EosWriteDownsampledViewWritesFullOrIssuesOnlyByPeriod) {
  eos::session::EosOutputDebugView view = MakeEosView();  // input_cycle_index = 1
  std::ostringstream out;

  // 非整周期（默认 period=10）：只落周期号 + 问题列表。
  EosWriteDownsampledView(view, out);
  EXPECT_EQ(out.str(),
            R"({"input_cycle_index":1,"issues":[{"severity":"error","phase":0,)"
            R"("code":"eos.verification","message":"synthetic","field":"dt_sec"}]})"
            "\n");

  // 整周期（10）：落全量帧（含全部目标行）。
  view.input_cycle_index = 10U;
  out.str("");
  EosWriteDownsampledView(view, out);
  EXPECT_NE(out.str().find("\"targets\":[{\"target_id\":42"), std::string::npos);
  EXPECT_NE(out.str().find("\"issues\":"), std::string::npos);

  // 自定义 period=2：周期 2 即全量帧。
  view.input_cycle_index = 2U;
  out.str("");
  EosWriteDownsampledView(view, out, 2U);
  EXPECT_NE(out.str().find("\"targets\":[{\"target_id\":42"), std::string::npos);

  // full_period=0 退化为每周期全量（避免除零 UB）。
  view.input_cycle_index = 3U;
  out.str("");
  EosWriteDownsampledView(view, out, 0U);
  EXPECT_NE(out.str().find("\"targets\":[{\"target_id\":42"), std::string::npos);
}

TEST(DebugViewJsonTest, ArWriteTrackStatusDeltasWritesOnlyChangedTracks) {
  std::unordered_map<std::uint64_t, ar::session::ArDebugTrackStatus> prev_status;
  std::ostringstream out;

  // 周期 7：两个目标均首次出现（external_target_id 作 key）→ 两行都写。
  ArWriteTrackStatusDeltas(
      MakeArViewWithStatuses(7U, ar::session::ArDebugTrackStatus::kConfirmed,
                             ar::session::ArDebugTrackStatus::kLost),
      prev_status, out);
  EXPECT_EQ(out.str(),
            R"({"external_target_id":10,"target_name":"a","status":"confirmed",)"
            R"("present_in_input":true,"has_track":false,"association_key":0,)"
            R"("position_x":0,"position_y":0,"position_z":0,"speed":0,"rcs":0,)"
            R"("hit_count":0,"miss_count":0,"target_type":""})"
            "\n"
            R"({"external_target_id":11,"target_name":"b","status":"lost",)"
            R"("present_in_input":true,"has_track":false,"association_key":0,)"
            R"("position_x":0,"position_y":0,"position_z":0,"speed":0,"rcs":0,)"
            R"("hit_count":0,"miss_count":0,"target_type":""})"
            "\n");

  // 周期 8：目标 10 保持 kConfirmed（不写），目标 11 变为 kTentative（写一行）。
  out.str("");
  ArWriteTrackStatusDeltas(
      MakeArViewWithStatuses(8U, ar::session::ArDebugTrackStatus::kConfirmed,
                             ar::session::ArDebugTrackStatus::kTentative),
      prev_status, out);
  EXPECT_NE(out.str().find("\"external_target_id\":11"), std::string::npos);
  EXPECT_EQ(out.str().find("\"external_target_id\":10"), std::string::npos);
  EXPECT_EQ(prev_status.at(10U), ar::session::ArDebugTrackStatus::kConfirmed);
  EXPECT_EQ(prev_status.at(11U), ar::session::ArDebugTrackStatus::kTentative);
}

TEST(DebugViewJsonTest, SbirsWriteNonNominalTargetsKeepsCoastingRows) {
  const sbirs::session::SbirsOutputDebugView view = MakeSbirsView();  // kCoasting
  std::ostringstream out;
  SbirsWriteNonNominalTargets(view, out);
  // kCoasting 不算标称（默认判定只跳过 kDetected）→ 该行照常落盘。
  EXPECT_NE(out.str().find("\"status\":\"coasting\""), std::string::npos);
}

TEST(DebugViewJsonTest, NominalStatusPredicates) {
  EXPECT_TRUE(EosIsNominalTargetStatus(eos::session::EosDebugTargetStatus::kDetected));
  EXPECT_FALSE(EosIsNominalTargetStatus(eos::session::EosDebugTargetStatus::kNotInOutput));
  EXPECT_TRUE(ArIsNominalTrackStatus(ar::session::ArDebugTrackStatus::kConfirmed));
  EXPECT_FALSE(ArIsNominalTrackStatus(ar::session::ArDebugTrackStatus::kTentative));
  EXPECT_TRUE(SbirsIsNominalTargetStatus(sbirs::session::SbirsDebugTargetStatus::kDetected));
  EXPECT_FALSE(SbirsIsNominalTargetStatus(sbirs::session::SbirsDebugTargetStatus::kCoasting));
}

TEST(DebugViewJsonTest, ArDebugTrackStateToJsonMatchesFullViewRow) {
  const ar::session::ArTrackOutputDebugView view = MakeArView();
  EXPECT_EQ(ArDebugTrackStateToJson(view.tracks[0]),
            R"({"external_target_id":42,"target_name":"tgt-\"A\"","status":"confirmed",)"
            R"("present_in_input":true,"has_track":true,"association_key":9,)"
            R"("position_x":1.5,"position_y":-2.5,"position_z":0,"speed":300,"rcs":2,)"
            R"("hit_count":4,"miss_count":1,"target_type":"threat"})");
}

TEST(DebugViewJsonTest, ArWriteNonNominalTracksSkipsConfirmedRows) {
  // 目标 10 为标称（kConfirmed）被跳过，目标 11 为丢失被写入。
  const ar::session::ArTrackOutputDebugView view =
      MakeArViewWithStatuses(7U, ar::session::ArDebugTrackStatus::kConfirmed,
                             ar::session::ArDebugTrackStatus::kLost);
  std::ostringstream out;
  ArWriteNonNominalTracks(view, out);
  EXPECT_EQ(out.str(),
            R"({"external_target_id":11,"target_name":"b","status":"lost",)"
            R"("present_in_input":true,"has_track":false,"association_key":0,)"
            R"("position_x":0,"position_y":0,"position_z":0,"speed":0,"rcs":0,)"
            R"("hit_count":0,"miss_count":0,"target_type":""})"
            "\n");
}

TEST(DebugViewJsonTest, ArWriteDownsampledViewWritesFullOrIssuesOnlyByPeriod) {
  // AR 周期字段为 world_cycle_index（uint64），与 EOS/SBIRS 的 input_cycle_index 不同，
  // 单独锁定该接线。
  const ar::session::ArTrackOutputDebugView view = MakeArView();  // world_cycle_index = 7
  std::ostringstream out;

  // 非整周期（默认 period=10）：只落周期号 + 问题列表。
  ArWriteDownsampledView(view, out);
  EXPECT_EQ(out.str(),
            R"({"world_cycle_index":7,"issues":[{"severity":"warning","phase":1,)"
            R"("code":"ar.impairment","message":"saturated"}]})"
            "\n");

  // 整周期（10）：落全量帧。
  ar::session::ArTrackOutputDebugView full_view = MakeArView();
  full_view.world_cycle_index = 10U;
  out.str("");
  ArWriteDownsampledView(full_view, out);
  EXPECT_NE(out.str().find("\"tracks\":[{\"external_target_id\":42"), std::string::npos);
  EXPECT_NE(out.str().find("\"issues\":"), std::string::npos);
}

TEST(DebugViewJsonTest, SbirsDebugTargetStateToJsonMatchesFullViewRow) {
  const sbirs::session::SbirsOutputDebugView view = MakeSbirsView();
  EXPECT_EQ(SbirsDebugTargetStateToJson(view.targets[0]),
            R"({"target_id":7,"target_name":"sat-7","status":"coasting",)"
            R"("present_in_input":true,"has_raw_output_record":true,"detected":true,)"
            R"("tracking_source":"estimated","estimated_range_m":1000,)"
            R"("has_estimation_nis":true,"estimation_nis":0.5,)"
            R"("estimation_nis_gate_exceeded":false,"nfov_channel_id":2,)"
            R"("has_nfov_tracking_diagnostics":true,"nfov_pointing_error_deg":0.1,)"
            R"("nfov_geometry_gate_passed":true,"nfov_snr_gate_passed":true,)"
            R"("nfov_tracking_gate_failure_count":3,"nfov_tracking_coasting":true,)"
            R"("azimuth_deg":45,"elevation_deg":30,"infrared_snr_linear":2,)"
            R"("observation_stage":"narrow_field_track"})");
}

TEST(DebugViewJsonTest, SbirsWriteTargetStatusDeltasWritesOnlyChangedTargets) {
  std::unordered_map<std::uint64_t, sbirs::session::SbirsDebugTargetStatus> prev_status;
  std::ostringstream out;

  // 周期 1：kCoasting 首次出现 → 写入。
  SbirsWriteTargetStatusDeltas(MakeSbirsView(), prev_status, out);
  EXPECT_NE(out.str().find("\"status\":\"coasting\""), std::string::npos);

  // 周期 2：同目标转为 kDetected → 状态变化 → 再写入。
  sbirs::session::SbirsOutputDebugView detected_view = MakeSbirsView();
  detected_view.input_cycle_index = 6U;
  detected_view.targets[0].status = sbirs::session::SbirsDebugTargetStatus::kDetected;
  out.str("");
  SbirsWriteTargetStatusDeltas(detected_view, prev_status, out);
  EXPECT_NE(out.str().find("\"status\":\"detected\""), std::string::npos);
  EXPECT_EQ(prev_status.at(7U), sbirs::session::SbirsDebugTargetStatus::kDetected);

  // 周期 3：状态未变 → 不写。
  out.str("");
  SbirsWriteTargetStatusDeltas(detected_view, prev_status, out);
  EXPECT_TRUE(out.str().empty());
}

TEST(DebugViewJsonTest, SbirsWriteDownsampledViewWritesFullOrIssuesOnlyByPeriod) {
  const sbirs::session::SbirsOutputDebugView view = MakeSbirsView();  // input_cycle_index = 5
  std::ostringstream out;

  // 非整周期（默认 period=10）：只落周期号 + 问题列表（SBIRS 视图 issues 为空）。
  SbirsWriteDownsampledView(view, out);
  EXPECT_EQ(out.str(), R"({"input_cycle_index":5,"issues":[]})" "\n");

  // 整周期（10）：落全量帧。
  sbirs::session::SbirsOutputDebugView full_view = MakeSbirsView();
  full_view.input_cycle_index = 10U;
  out.str("");
  SbirsWriteDownsampledView(full_view, out);
  EXPECT_NE(out.str().find("\"targets\":[{\"target_id\":7"), std::string::npos);
  EXPECT_NE(out.str().find("\"issues\":[]"), std::string::npos);
}
