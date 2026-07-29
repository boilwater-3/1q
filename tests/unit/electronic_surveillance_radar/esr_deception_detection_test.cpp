#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

// 手工构造仅含给定方位角脉冲观测的 InterceptDetectionOutput。
// 所有观测的 waveform_class = kPulse。
InterceptDetectionOutput MakePulseOutput(std::vector<double> azimuths) {
  InterceptDetectionOutput output;
  for (double az : azimuths) {
    RawObservationRecord rec;
    rec.observation.aoa_az_deg = az;
    rec.observation.aoa_el_deg = 0.0;
    rec.observation.waveform_class = session::EsrWaveformClass::kPulse;
    output.raw_records.push_back(rec);
  }
  return output;
}

// 构造含非脉冲观测和脉冲观测混合的输出。
InterceptDetectionOutput MakeMixedOutput(
    std::vector<session::EsrWaveformClass> waveform_classes,
    std::vector<double> azimuths) {
  InterceptDetectionOutput output;
  for (std::size_t i = 0U; i < waveform_classes.size(); ++i) {
    RawObservationRecord rec;
    rec.observation.aoa_az_deg = i < azimuths.size() ? azimuths[i] : 0.0;
    rec.observation.aoa_el_deg = 0.0;
    rec.observation.waveform_class = waveform_classes[i];
    output.raw_records.push_back(rec);
  }
  return output;
}

// 单个脉冲观测 → kNone
TEST(EsrDeceptionDetectionTest, SinglePulseIsNotDeceptive) {
  auto output = MakePulseOutput({0.0});
  ClassifyDeception(10.0f, 10.0f, &output);
  ASSERT_EQ(output.raw_records.size(), 1U);
  EXPECT_EQ(output.raw_records.front().observation.deception_class,
            session::EsrDeceptionClass::kNone);
}

// 同波束宽度内两个脉冲观测 → kLikelyFalseTarget
TEST(EsrDeceptionDetectionTest, TwoPulsesWithinBeamwidthAreDeceptive) {
  auto output = MakePulseOutput({0.0, 5.0});
  ClassifyDeception(10.0f, 10.0f, &output);
  ASSERT_EQ(output.raw_records.size(), 2U);
  EXPECT_EQ(output.raw_records[0].observation.deception_class,
            session::EsrDeceptionClass::kLikelyFalseTarget);
  EXPECT_EQ(output.raw_records[1].observation.deception_class,
            session::EsrDeceptionClass::kLikelyFalseTarget);
}

// 非脉冲波形不参与欺骗检测
TEST(EsrDeceptionDetectionTest, NonPulseWaveformsDoNotTriggerDeception) {
  auto output = MakeMixedOutput(
      {session::EsrWaveformClass::kNoise, session::EsrWaveformClass::kPulse},
      {0.0, 0.0});
  ClassifyDeception(120.0f, 120.0f, &output);
  ASSERT_EQ(output.raw_records.size(), 2U);
  EXPECT_EQ(output.raw_records[0].observation.deception_class,
            session::EsrDeceptionClass::kNone);
  EXPECT_EQ(output.raw_records[1].observation.deception_class,
            session::EsrDeceptionClass::kNone);
}

// 方位角差超过波束宽度不标记
TEST(EsrDeceptionDetectionTest, SeparatedPulsesNotDeceptive) {
  auto output = MakePulseOutput({0.0, 20.0});
  ClassifyDeception(10.0f, 10.0f, &output);
  ASSERT_EQ(output.raw_records.size(), 2U);
  EXPECT_EQ(output.raw_records[0].observation.deception_class,
            session::EsrDeceptionClass::kNone);
  EXPECT_EQ(output.raw_records[1].observation.deception_class,
            session::EsrDeceptionClass::kNone);
}

// 方位角相近但俯仰角超出波束宽度不标记（el_diff 独立检查）
TEST(EsrDeceptionDetectionTest, ElevationSeparationPreventsDeception) {
  auto output = MakePulseOutput({0.0, 0.0});
  output.raw_records[0].observation.aoa_el_deg = 0.0;
  output.raw_records[1].observation.aoa_el_deg = 15.0;
  ClassifyDeception(10.0f, 10.0f, &output);
  ASSERT_EQ(output.raw_records.size(), 2U);
  EXPECT_EQ(output.raw_records[0].observation.deception_class,
            session::EsrDeceptionClass::kNone);
  EXPECT_EQ(output.raw_records[1].observation.deception_class,
            session::EsrDeceptionClass::kNone);
}

// nullptr 安全
TEST(EsrDeceptionDetectionTest, NullOutputIsSafe) {
  ClassifyDeception(10.0f, 10.0f, nullptr);
}

// 空观测列表安全
TEST(EsrDeceptionDetectionTest, EmptyObservationsIsSafe) {
  InterceptDetectionOutput output;
  ClassifyDeception(10.0f, 10.0f, &output);
  EXPECT_TRUE(output.raw_records.empty());
}

// 三个同向脉冲全部标记
TEST(EsrDeceptionDetectionTest, ThreePulsesWithinBeamwidthAllDeceptive) {
  auto output = MakePulseOutput({0.0, 3.0, 6.0});
  ClassifyDeception(10.0f, 10.0f, &output);
  ASSERT_EQ(output.raw_records.size(), 3U);
  for (const auto& rec : output.raw_records) {
    EXPECT_EQ(rec.observation.deception_class,
              session::EsrDeceptionClass::kLikelyFalseTarget);
  }
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
