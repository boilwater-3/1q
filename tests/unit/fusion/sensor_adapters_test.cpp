// Copyright 2026. All Rights Reserved.
//
// @file sensor_adapters_test.cpp
// @brief 传感器输出 → 融合探测记录官方适配器（SensorAdapters）单元测试。
//
// 覆盖四传感器适配函数：正常映射全字段、跳过规则（AR kLost / ESR
// hypothesis_id==0 / EOS-SBIRS detected==false）、质量归一化夹取、AR ECEF
// 转换失败退化分支、source_id 透传、空输入 → 空输出。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/fusion/SensorAdapters.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace fusion {
namespace {

// ============================ AR 轨迹帧构造 ============================

airborne_radar::session::ArExternalTrackKinematics MakeArTrack(
    std::uint64_t key, airborne_radar::session::TrackStatus status,
    const oneq::coordinate::EcefPositionM& ecef, float probability) {
  airborne_radar::session::ArExternalTrackKinematics track;
  track.association_key = key;
  track.status = status;
  track.target_position_ecef_m = ecef;
  track.target_probability = probability;
  return track;
}

/// 由已知 LLA 反向生成 ECEF（测试数据用真实坐标换算；换算失败即测试前提不成立）。
oneq::coordinate::EcefPositionM MakeEcefFromLla(double lat_deg, double lon_deg,
                                                double alt_m) {
  oneq::coordinate::LlaPositionDegM lla;
  lla.latitude_deg = lat_deg;
  lla.longitude_deg = lon_deg;
  lla.altitude_m = alt_m;
  oneq::coordinate::EcefPositionM ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(lla, &ecef));
  return ecef;
}

/// 测试基准位置（机场 30°N, 120°E，400 m 高）：多个 AR 用例共用。
const oneq::coordinate::EcefPositionM kTestEcef = MakeEcefFromLla(30.0, 120.0, 400.0);

// ============================ ESR 假设构造 ============================

electronic_surveillance_radar::session::EmitterHypothesis MakeHypothesis(
    std::uint64_t id, double freq_hz, double bandwidth_hz, double pri_s,
    double pulse_width_s, float confidence) {
  electronic_surveillance_radar::session::EmitterHypothesis hypothesis;
  hypothesis.hypothesis_id = id;
  hypothesis.bearing_az_deg = 45.0f;
  hypothesis.bearing_el_deg = 10.0f;
  hypothesis.estimated_center_frequency_hz = freq_hz;
  hypothesis.estimated_bandwidth_hz = bandwidth_hz;
  hypothesis.estimated_pri_s = pri_s;
  hypothesis.estimated_pulse_width_s = pulse_width_s;
  hypothesis.confidence = confidence;
  return hypothesis;
}

// ============================ EOS/SBIRS 探测构造 ============================

electro_optical_sensor::output::EosDetectionRecord MakeEosRecord(bool detected,
                                                                 float snr_db) {
  electro_optical_sensor::output::EosDetectionRecord record;
  record.azimuth_deg = 90.0f;
  record.elevation_deg = 5.0f;
  record.fused_snr_db = snr_db;
  record.detected = detected;
  return record;
}

sbirs_sensor::output::SbirsDetectionRecord MakeSbirsRecord(bool detected,
                                                           float ir_snr_linear) {
  sbirs_sensor::output::SbirsDetectionRecord record;
  // SBIRS 输出为 ECI 弧度（2026-08）：120° ≈ 2.0944 rad、−30° ≈ −0.5236 rad。
  record.azimuth_rad = 2.0943951023931953f;
  record.elevation_rad = -0.5235987755982988f;
  record.infrared_snr_linear = ir_snr_linear;
  record.detected = detected;
  return record;
}

// ============================ AR 适配 ============================

TEST(SensorAdaptersTest, ArTracksAdaptToDetectionRecords) {
  const oneq::coordinate::EcefPositionM& ecef = kTestEcef;
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {MakeArTrack(1001U, airborne_radar::session::TrackStatus::kConfirmed,
                              ecef, 0.8f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(kArSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& record = detections[0];
  EXPECT_EQ(record.key, 1001U);
  EXPECT_EQ(record.source_id, kArSourceId);
  EXPECT_TRUE(record.has_position);
  EXPECT_NEAR(record.position.latitude_deg, 30.0, 1e-6);
  EXPECT_NEAR(record.position.longitude_deg, 120.0, 1e-6);
  EXPECT_NEAR(record.position.altitude_m, 400.0, 1.0);
  EXPECT_DOUBLE_EQ(record.verdict, 1.0);
  EXPECT_NEAR(record.quality, 0.8, 1e-6);  // target_probability 透传（float 源）
}

TEST(SensorAdaptersTest, ArLostTrackIsSkipped) {
  const oneq::coordinate::EcefPositionM& ecef = kTestEcef;
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {
      MakeArTrack(1001U, airborne_radar::session::TrackStatus::kConfirmed, ecef, 0.8f),
      MakeArTrack(1002U, airborne_radar::session::TrackStatus::kLost, ecef, 0.0f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(kArSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].key, 1001U);
}

TEST(SensorAdaptersTest, ArZeroProbabilityUsesStatusBaseQuality) {
  const oneq::coordinate::EcefPositionM& ecef = kTestEcef;
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {
      MakeArTrack(1001U, airborne_radar::session::TrackStatus::kConfirmed, ecef, 0.0f),
      MakeArTrack(1002U, airborne_radar::session::TrackStatus::kTentative, ecef, 0.0f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(kArSourceId, frame);

  ASSERT_EQ(detections.size(), 2U);
  EXPECT_DOUBLE_EQ(detections[0].quality, 1.0);  // kConfirmed → 1.0
  EXPECT_DOUBLE_EQ(detections[1].quality, 0.5);  // kTentative → 0.5
}

TEST(SensorAdaptersTest, ArEcefConversionFailureDegradesToIdentityKeyOnly) {
  // 全零 ECEF：TryEcefToLla 对 norm ≤ floor 返回 false → has_position=false。
  const oneq::coordinate::EcefPositionM zero_ecef{};
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {MakeArTrack(1001U, airborne_radar::session::TrackStatus::kConfirmed,
                              zero_ecef, 0.8f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(kArSourceId, frame);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].key, 1001U);
  EXPECT_FALSE(detections[0].has_position);
  EXPECT_NEAR(detections[0].quality, 0.8, 1e-6);  // 退化不影响质量（float 源）
}

// ============================ ESR 适配 ============================

TEST(SensorAdaptersTest, EsrHypothesesAdaptToDetectionRecords) {
  electronic_surveillance_radar::session::EmitterHypothesisList hypotheses;
  // 9.5 GHz / 2 MHz / 1 ms PRI / 1 µs 脉宽。
  hypotheses.push_back(MakeHypothesis(7U, 9.5e9, 2.0e6, 1.0e-3, 1.0e-6, 0.9f));

  const std::vector<DetectionRecord> detections =
      AdaptEsrHypothesesToDetectionRecords(kEsrSourceId, hypotheses);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& record = detections[0];
  EXPECT_EQ(record.key, 7U);
  EXPECT_EQ(record.source_id, kEsrSourceId);
  EXPECT_TRUE(record.has_bearing);
  EXPECT_DOUBLE_EQ(record.bearing_az_deg, 45.0);
  EXPECT_DOUBLE_EQ(record.bearing_el_deg, 10.0);
  // 特征归一化标尺：GHz / MHz / ms / µs。
  ASSERT_EQ(record.feature.size(), 4U);
  EXPECT_DOUBLE_EQ(record.feature[0], 9.5);
  EXPECT_DOUBLE_EQ(record.feature[1], 2.0);
  EXPECT_DOUBLE_EQ(record.feature[2], 1.0);
  EXPECT_DOUBLE_EQ(record.feature[3], 1.0);
  EXPECT_DOUBLE_EQ(record.verdict, 1.0);
  EXPECT_NEAR(record.quality, 0.9, 1e-6);  // confidence 透传（float 源）
}

TEST(SensorAdaptersTest, EsrZeroHypothesisIdIsSkipped) {
  electronic_surveillance_radar::session::EmitterHypothesisList hypotheses;
  hypotheses.push_back(MakeHypothesis(0U, 9.5e9, 2.0e6, 1.0e-3, 1.0e-6, 0.9f));
  hypotheses.push_back(MakeHypothesis(8U, 10.0e9, 1.0e6, 5.0e-4, 2.0e-6, 0.7f));

  const std::vector<DetectionRecord> detections =
      AdaptEsrHypothesesToDetectionRecords(kEsrSourceId, hypotheses);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].key, 8U);
}

// ============================ EOS 适配 ============================

TEST(SensorAdaptersTest, EosDetectionsAdaptToDetectionRecords) {
  electro_optical_sensor::output::EosDetectionRecordList records;
  records.push_back(MakeEosRecord(/*detected=*/true, /*snr_db=*/20.0f));

  const std::vector<DetectionRecord> detections =
      AdaptEosDetectionsToDetectionRecords(kEosSourceId, records);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& record = detections[0];
  EXPECT_EQ(record.key, 0U);  // 无身份 → 走方位相干关联
  EXPECT_EQ(record.source_id, kEosSourceId);
  EXPECT_TRUE(record.has_bearing);
  EXPECT_DOUBLE_EQ(record.bearing_az_deg, 90.0);
  EXPECT_DOUBLE_EQ(record.bearing_el_deg, 5.0);
  EXPECT_DOUBLE_EQ(record.verdict, 1.0);
  EXPECT_DOUBLE_EQ(record.quality, 1.0);  // 20 dB / 10 → 2.0 夹取到 1.0
}

TEST(SensorAdaptersTest, EosQualityClampedBySnrNormalization) {
  electro_optical_sensor::output::EosDetectionRecordList records;
  records.push_back(MakeEosRecord(/*detected=*/true, /*snr_db=*/5.0f));
  records.push_back(MakeEosRecord(/*detected=*/true, /*snr_db=*/-5.0f));

  const std::vector<DetectionRecord> detections =
      AdaptEosDetectionsToDetectionRecords(kEosSourceId, records);

  ASSERT_EQ(detections.size(), 2U);
  EXPECT_DOUBLE_EQ(detections[0].quality, 0.5);  // 5 dB / 10
  EXPECT_DOUBLE_EQ(detections[1].quality, 0.0);  // -5 dB / 10 夹取到 0
}

TEST(SensorAdaptersTest, EosNotDetectedIsSkipped) {
  electro_optical_sensor::output::EosDetectionRecordList records;
  records.push_back(MakeEosRecord(/*detected=*/false, /*snr_db=*/10.0f));

  const std::vector<DetectionRecord> detections =
      AdaptEosDetectionsToDetectionRecords(kEosSourceId, records);

  EXPECT_TRUE(detections.empty());
}

// ============================ SBIRS 适配 ============================

TEST(SensorAdaptersTest, SbirsDetectionsAdaptToDetectionRecords) {
  sbirs_sensor::output::SbirsDetectionRecordList records;
  records.push_back(MakeSbirsRecord(/*detected=*/true, /*ir_snr_linear=*/8.0f));

  const std::vector<DetectionRecord> detections =
      AdaptSbirsDetectionsToDetectionRecords(kSbirsSourceId, records);

  ASSERT_EQ(detections.size(), 1U);
  const DetectionRecord& record = detections[0];
  EXPECT_EQ(record.key, 0U);  // 无身份 → 走方位相干关联
  EXPECT_EQ(record.source_id, kSbirsSourceId);
  EXPECT_TRUE(record.has_bearing);
  // 2.0943951f rad × rad2deg（float 运算）≈ 120.000008°。
  EXPECT_NEAR(record.bearing_az_deg, 120.0, 1.0e-4);
  EXPECT_NEAR(record.bearing_el_deg, -30.0, 1.0e-4);
  EXPECT_DOUBLE_EQ(record.verdict, 1.0);
  EXPECT_DOUBLE_EQ(record.quality, 1.0);  // 8.0 / 4 → 2.0 夹取到 1.0
}

TEST(SensorAdaptersTest, SbirsQualityNormalizedToWfovThreshold) {
  sbirs_sensor::output::SbirsDetectionRecordList records;
  records.push_back(MakeSbirsRecord(/*detected=*/true, /*ir_snr_linear=*/2.0f));
  records.push_back(MakeSbirsRecord(/*detected=*/true, /*ir_snr_linear=*/-1.0f));

  const std::vector<DetectionRecord> detections =
      AdaptSbirsDetectionsToDetectionRecords(kSbirsSourceId, records);

  ASSERT_EQ(detections.size(), 2U);
  EXPECT_DOUBLE_EQ(detections[0].quality, 0.5);  // 2.0 / 4
  EXPECT_DOUBLE_EQ(detections[1].quality, 0.0);  // 负值夹取到 0
}

TEST(SensorAdaptersTest, SbirsNotDetectedIsSkipped) {
  sbirs_sensor::output::SbirsDetectionRecordList records;
  records.push_back(MakeSbirsRecord(/*detected=*/false, /*ir_snr_linear=*/4.0f));

  const std::vector<DetectionRecord> detections =
      AdaptSbirsDetectionsToDetectionRecords(kSbirsSourceId, records);

  EXPECT_TRUE(detections.empty());
}

// ============================ 通用 ============================

TEST(SensorAdaptersTest, SourceIdPassedThrough) {
  // 验证调用方自定义 source_id 透传（不强制使用常量）。
  const oneq::coordinate::EcefPositionM& ecef = kTestEcef;
  airborne_radar::session::ArExternalTrackOutputFrame frame;
  frame.tracks = {MakeArTrack(1U, airborne_radar::session::TrackStatus::kConfirmed,
                              ecef, 0.5f)};

  const std::vector<DetectionRecord> detections =
      AdaptArTracksToDetectionRecords(42U, frame);

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections[0].source_id, 42U);
}

TEST(SensorAdaptersTest, EmptyInputYieldsEmptyOutput) {
  airborne_radar::session::ArExternalTrackOutputFrame ar_frame;
  EXPECT_TRUE(AdaptArTracksToDetectionRecords(kArSourceId, ar_frame).empty());

  electronic_surveillance_radar::session::EmitterHypothesisList esr_hypotheses;
  EXPECT_TRUE(AdaptEsrHypothesesToDetectionRecords(kEsrSourceId, esr_hypotheses).empty());

  electro_optical_sensor::output::EosDetectionRecordList eos_records;
  EXPECT_TRUE(AdaptEosDetectionsToDetectionRecords(kEosSourceId, eos_records).empty());

  sbirs_sensor::output::SbirsDetectionRecordList sbirs_records;
  EXPECT_TRUE(AdaptSbirsDetectionsToDetectionRecords(kSbirsSourceId, sbirs_records).empty());
}

}  // namespace
}  // namespace fusion
