/**
 * @file debug_view_json_test.cpp
 * @brief 锁定 4 模块 DebugView → JSON 序列化输出格式（规则 12 参考实现）。
 *
 * 覆盖要点：
 *   - AR/EOS/SAR/SBIRS 四个模块序列化器对完整手填视图的逐字 JSON 输出断言；
 *   - JSON 转义（引号/反斜杠/换行/控制字符）；
 *   - 同时证明恢复的 AR/EOS 序列化器能针对当前 DebugView 结构编译。
 *
 * 对应契约 docs/common/session_contract.md 三层输出模型规则 12 的参考实现
 * （examples/common/*DebugViewToJson.h + debug_view_json.h）。
 */

#include <gtest/gtest.h>

#include <string>

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
  view.has_error = false;
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
                  R"("has_error":false,"abort_reason":"","completed_stage":"l3_bp_image",)"
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
