// Copyright 2026. All Rights Reserved.
//
// @file ar_rir_recognition_equivalence_test.cpp
// @brief AR kLrr 识别链路与新模块 RIR 的数值等价性对比（阶段 1 步骤 7 保真守卫）。
//
// 同场景双跑：AR（启用识别 + kLrr patch）与 RIR（kIdentify + 同库同策略）各 5 周期。
// RIR 输入由 AR 自身输出构造：航迹供给逐字段取自 AR 公开 TrackOutputFrame，
// 场景目标位置用公共 TryEcefToEnu 复算 AR 内部雷达局部系（平台姿态恒等）。
// 断言：识别结论逐字段一致（浮点容差 1e-5f）。本测试只读 AR 公开 API。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RecognitionSqliteTestUtil.h"
#include "RirSqliteTestUtil.h"

namespace airborne_radar {
namespace tests {
namespace {

// RIR 域命名空间别名（本测试位于 airborne_radar::tests，跨模块引用需别名）。
namespace rir_config = remote_identification_radar::config;
namespace rir_session = remote_identification_radar::session;

using session::ArCycleInput;
using session::ArCycleResult;
using session::ArCycleStatus;
using session::ArRecognitionCategory;
using session::ArRecognitionResult;
using session::ArRecognitionState;
using session::ArSession;
using session::ArTargetInput;
using session::TrackStatus;
using rir_session::RirCycleInput;
using rir_session::RirCycleResult;
using rir_session::RirCycleStatus;
using rir_session::RirRecognitionResult;
using rir_session::RirRecognitionState;
using rir_session::RirSceneTarget;
using rir_session::RirSession;
using rir_session::RirTrackFeedEntry;
using rir_session::RirTrackFeedStatus;

constexpr const char* kEquivalenceDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','rir-equivalence-baseline'),
  ('version','2.0.0'),
  ('created_utc','2026-07-22T00:00:00Z'),
  ('polarization_channels','H,V'),
  ('polarization_energy_reference','range_propagation_antenna_compensated');
INSERT INTO units VALUES
  ('rcs','dBsm'),('speed','m/s'),('altitude','m'),('acceleration','m/s2'),
  ('turn_radius','m'),('polarization','dB'),('range','m');
INSERT INTO categories VALUES ('BALLISTIC','弹道目标',0.5), ('NEAR_SPACE','临近空间目标',0.5);
INSERT INTO models VALUES
  ('BALLISTIC_EXAMPLE_A','BALLISTIC','弹道目标示例 A',1.0),
  ('NEAR_SPACE_EXAMPLE_A','NEAR_SPACE','临近空间目标示例 A',1.0);
INSERT INTO profiles VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',-30.0,NULL,NULL,NULL,NULL,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',-30.0,NULL,NULL,NULL,NULL,NULL);
INSERT INTO rcs_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',-3.0,2.0,NULL,NULL,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',2.0,2.5,NULL,NULL,NULL);
INSERT INTO motion_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',100.0,30.0,3000.0,500.0,0.0,6.0,6.0,0.5),
  ('nominal','NEAR_SPACE_EXAMPLE_A',400.0,80.0,1500.0,300.0,0.0,2.0,4.5,0.5);
)sql";

class ArRirRecognitionEquivalenceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    database_path_ =
        WriteTempSqlite("ar_rir_equivalence.db",
                        std::string(kRecognitionSchemaSql) + kEquivalenceDatabaseSql);
    ASSERT_FALSE(database_path_.empty());
  }

  void SetRecognitionPolicy(airborne_radar::config::ArRecognitionConfig* policy) const {
    policy->enabled = true;
    policy->database_path = database_path_;
    policy->min_confirmed_hits = 1U;
    policy->min_observation_count = 1U;
    policy->acceptance_score = 0.6f;
    policy->minimum_margin = 0.05f;
    policy->result_hold_sec = 1.0f;
  }

  void SetRecognitionPolicy(rir_config::RirRecognitionPolicy* policy) const {
    policy->enabled = true;
    policy->database_path = database_path_;
    policy->min_confirmed_hits = 1U;
    policy->min_observation_count = 1U;
    policy->acceptance_score = 0.6f;
    policy->minimum_margin = 0.05f;
    policy->result_hold_sec = 1.0f;
  }

  static void AppendAspectSamples(std::vector<session::AspectRcsSample>* samples,
                                  float rcs_dbsm) {
    for (float az = -5.0f; az <= 5.0f; az += 5.0f) {
      for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
        session::AspectRcsSample aspect;
        aspect.aspect_az_deg = az;
        aspect.aspect_el_deg = el;
        aspect.rcs_dbsm = rcs_dbsm;
        samples->push_back(aspect);
      }
    }
  }

  static void AppendAspectSamples(std::vector<rir_session::RirAspectRcsSample>* samples,
                                  float rcs_dbsm) {
    for (float az = -5.0f; az <= 5.0f; az += 5.0f) {
      for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
        rir_session::RirAspectRcsSample aspect;
        aspect.aspect_az_deg = az;
        aspect.aspect_el_deg = el;
        aspect.rcs_dbsm = rcs_dbsm;
        samples->push_back(aspect);
      }
    }
  }

  std::string database_path_{};
};

TEST_F(ArRirRecognitionEquivalenceTest, BallisticScenarioMatchesFieldByField) {
  // -- AR 侧会话（只读公开 API） --
  airborne_radar::config::ArSessionConfig ar_config;
  ar_config.policy.detection = airborne_radar::config::profiles::kDetectionPriorityDetection;
  ar_config.policy.tracking = airborne_radar::config::profiles::kFastAssociationTracking;
  ar_config.policy.tracking.enable_kalman_filter = true;
  ar_config.policy.lifecycle = airborne_radar::config::profiles::kFastConfirmLifecycle;
  SetRecognitionPolicy(&ar_config.policy.recognition);
  ArSession ar_radar = ArSession::Create(ar_config);
  airborne_radar::config::ArRuntimeConfigPatch lrr_patch;
  lrr_patch.has_work_mode = true;
  lrr_patch.work_mode = airborne_radar::config::ArWorkMode::kLrr;
  ASSERT_TRUE(ar_radar.TryApplyRuntimeConfig(lrr_patch));

  // -- RIR 侧会话（默认硬件与 AR 默认硬件数值一致） --
  rir_config::RirSessionConfig rir_config;
  rir_config.mission.work_mode = rir_config::RirWorkMode::kIdentify;
  SetRecognitionPolicy(&rir_config.policy.recognition);
  RirSession rir_radar = RirSession::Create(rir_config);

  // -- 平台几何：lat 31 / lon 121 / alt 1000（与 AR 场景测试一致） --
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1000.0;
  oneq::coordinate::EcefPositionM platform_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(platform_lla, &platform_ecef));

  constexpr float kSpeedMps = 100.0f;
  constexpr float kAltitudeOffsetM = 2000.0f;
  constexpr float kRcsDbsm = -3.0f;
  constexpr char kTruthName[] = "BALLISTIC_EXAMPLE_A";
  const std::uint64_t kTargetId = 77U;
  constexpr std::uint32_t kCycles = 5U;
  constexpr double kDtSec = 0.5;

  for (std::uint32_t cycle = 1U; cycle <= kCycles; ++cycle) {
    // -- AR 输入 --
    ArCycleInput ar_input;
    ar_input.cycle_index = cycle;
    ar_input.cycle_start_time_s = static_cast<double>(cycle - 1U) * kDtSec;
    ar_input.dt_sec = kDtSec;
    ar_input.platform.platform_entity_id = 10U;
    ar_input.platform.platform_position_ecef_m = platform_ecef;

    ArTargetInput ar_target;
    ar_target.target_id = kTargetId;
    ar_target.target_name = kTruthName;
    ar_target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    ar_target.kinematics.position_ecef_m = platform_ecef;
    ar_target.kinematics.position_ecef_m.x_m +=
        5000.0 + kSpeedMps * static_cast<double>(cycle - 1U) * kDtSec;
    ar_target.kinematics.position_ecef_m.z_m += kAltitudeOffsetM;
    ar_target.kinematics.velocity_mps.x_mps = kSpeedMps;
    ar_target.rcs = 5.0f;
    AppendAspectSamples(&ar_target.aspect_rcs_samples, kRcsDbsm);
    ar_input.targets.push_back(ar_target);

    const ArCycleResult ar_result = ar_radar.StepWithResult(ar_input);
    ASSERT_EQ(ar_result.status, ArCycleStatus::kCompleted) << "cycle " << cycle;
    ASSERT_EQ(ar_result.output_frame.tracks.size(), 1U) << "cycle " << cycle;
    const auto& ar_track = ar_result.output_frame.tracks.front();

    // -- RIR 输入（由 AR 输出 + 公共坐标变换构造） --
    RirCycleInput rir_input;
    rir_input.input_cycle_index = cycle;
    // AR 内部批号随成功周期递增（1..N），RIR 侧由调用方供给，喂同一序列以对齐
    // 结论的 source_batch_id 溯源字段。
    rir_input.batch_id = cycle;
    rir_input.dt_sec = kDtSec;
    rir_input.sim_time_sec = static_cast<float>(cycle - 1U) * static_cast<float>(kDtSec);
    rir_input.platform_altitude_m = 1000.0f;

    // 场景目标：雷达局部系 = AR 内部 ENU（平台姿态恒等），范围 = |ENU|。
    oneq::coordinate::EnuPositionM target_enu;
    ASSERT_TRUE(oneq::coordinate::TryEcefToEnu(ar_target.kinematics.position_ecef_m,
                                               platform_lla, &target_enu));
    RirSceneTarget rir_target;
    rir_target.external_target_id = kTargetId;
    rir_target.position_x = static_cast<float>(target_enu.east_m);
    rir_target.position_y = static_cast<float>(target_enu.north_m);
    rir_target.position_z = static_cast<float>(target_enu.up_m);
    rir_target.range_m = static_cast<float>(
        std::sqrt(target_enu.east_m * target_enu.east_m + target_enu.north_m * target_enu.north_m +
                  target_enu.up_m * target_enu.up_m));
    rir_target.rcs = 5.0f;
    AppendAspectSamples(&rir_target.aspect_rcs_samples, kRcsDbsm);
    rir_input.scene_targets.push_back(rir_target);

    // 航迹供给：逐字段投影 AR 公开航迹输出。
    RirTrackFeedEntry feed;
    feed.association_key = ar_track.association_key;
    feed.external_target_id = ar_track.external_target_id;
    feed.target_name = ar_track.target_name;
    feed.status = ar_track.status == TrackStatus::kConfirmed ? RirTrackFeedStatus::kConfirmed
                                                             : RirTrackFeedStatus::kTentative;
    feed.hit_count = ar_track.hit_count;
    feed.position_x = ar_track.position_x;
    feed.position_y = ar_track.position_y;
    feed.position_z = ar_track.position_z;
    feed.velocity_x = ar_track.velocity_x;
    feed.velocity_y = ar_track.velocity_y;
    feed.velocity_z = ar_track.velocity_z;
    feed.speed = ar_track.speed;
    feed.acceleration_x = ar_track.acceleration_x;
    feed.acceleration_y = ar_track.acceleration_y;
    feed.acceleration_z = ar_track.acceleration_z;
    feed.acceleration = ar_track.acceleration;
    feed.estimation_uncertainty_trace = ar_track.estimation_uncertainty_trace;
    feed.target_type = ar_track.target_type;
    rir_input.track_feed.push_back(feed);

    const RirCycleResult rir_result = rir_radar.StepWithResult(rir_input);
    ASSERT_EQ(rir_result.status, RirCycleStatus::kCompleted) << "cycle " << cycle;
    ASSERT_EQ(rir_result.output_frame.recognition_outputs.size(), 1U) << "cycle " << cycle;
    const auto& rir_output = rir_result.output_frame.recognition_outputs.front();

    // -- 逐字段等价断言（浮点容差 1e-5f） --
    const ArRecognitionResult& ar_rec = ar_track.recognition;
    const RirRecognitionResult& rir_rec = rir_output.result;
    EXPECT_EQ(static_cast<int>(rir_rec.state), static_cast<int>(ar_rec.state)) << "cycle " << cycle;
    EXPECT_EQ(static_cast<int>(rir_rec.target_category),
              static_cast<int>(ar_rec.target_category))
        << "cycle " << cycle;
    EXPECT_EQ(rir_rec.target_model, ar_rec.target_model) << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.confidence, ar_rec.confidence, 1.0e-5f) << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.best_score, ar_rec.best_score, 1.0e-5f) << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.runner_up_score, ar_rec.runner_up_score, 1.0e-5f) << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.feature_scores.rcs_similarity, ar_rec.feature_scores.rcs_similarity,
                1.0e-5f)
        << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.feature_scores.rcs_quality, ar_rec.feature_scores.rcs_quality, 1.0e-5f)
        << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.feature_scores.motion_similarity, ar_rec.feature_scores.motion_similarity,
                1.0e-5f)
        << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.feature_scores.motion_quality, ar_rec.feature_scores.motion_quality,
                1.0e-5f)
        << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.feature_scores.polarization_similarity,
                ar_rec.feature_scores.polarization_similarity, 1.0e-5f)
        << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.feature_scores.polarization_quality,
                ar_rec.feature_scores.polarization_quality, 1.0e-5f)
        << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.feature_scores.range_profile_similarity,
                ar_rec.feature_scores.range_profile_similarity, 1.0e-5f)
        << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.feature_scores.range_profile_quality,
                ar_rec.feature_scores.range_profile_quality, 1.0e-5f)
        << "cycle " << cycle;
    EXPECT_EQ(rir_rec.valid_feature_mask, ar_rec.valid_feature_mask) << "cycle " << cycle;
    EXPECT_EQ(rir_rec.observation_count, ar_rec.observation_count) << "cycle " << cycle;
    EXPECT_NEAR(rir_rec.accumulation_sec, ar_rec.accumulation_sec, 1.0e-5f) << "cycle " << cycle;
    EXPECT_EQ(rir_rec.database_version, ar_rec.database_version) << "cycle " << cycle;
    EXPECT_EQ(rir_rec.source_cycle_index, ar_rec.source_cycle_index) << "cycle " << cycle;
    EXPECT_EQ(rir_rec.source_batch_id, ar_rec.source_batch_id) << "cycle " << cycle;

    // 摘要一致性。
    EXPECT_EQ(rir_result.has_recognition_summary, ar_result.has_recognition_summary);
    if (ar_result.has_recognition_summary) {
      EXPECT_EQ(rir_result.recognition_summary.participating_track_count,
                ar_result.recognition_summary.participating_track_count);
      EXPECT_EQ(rir_result.recognition_summary.model_confirmed_count,
                ar_result.recognition_summary.model_confirmed_count);
      EXPECT_NEAR(rir_result.recognition_summary.mean_confidence,
                  ar_result.recognition_summary.mean_confidence, 1.0e-5f);
    }
  }
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
