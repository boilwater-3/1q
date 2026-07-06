#include <gtest/gtest.h>

#include <type_traits>

#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace {

template <typename T>
class HasRangeM {
  typedef char Yes[1];
  typedef char No[2];
  template <typename U>
  static Yes& Test(decltype(&U::range_m));
  template <typename>
  static No& Test(...);

 public:
  static const bool value = sizeof(Test<T>(0)) == sizeof(Yes);
};

template <typename T>
class HasVisibleSnr {
  typedef char Yes[1];
  typedef char No[2];
  template <typename U>
  static Yes& Test(decltype(&U::visible_snr_linear));
  template <typename>
  static No& Test(...);

 public:
  static const bool value = sizeof(Test<T>(0)) == sizeof(Yes);
};

template <typename T>
class HasFusedSnr {
  typedef char Yes[1];
  typedef char No[2];
  template <typename U>
  static Yes& Test(decltype(&U::fused_snr_linear));
  template <typename>
  static No& Test(...);

 public:
  static const bool value = sizeof(Test<T>(0)) == sizeof(Yes);
};

TEST(SbirsOutputBoundaryContractTest, RawDetectionExcludesEosCompatibilityFields) {
  typedef sbirs_sensor::output::SbirsDetectionRecord Record;
  EXPECT_FALSE(HasRangeM<Record>::value);
  EXPECT_FALSE(HasVisibleSnr<Record>::value);
  EXPECT_FALSE(HasFusedSnr<Record>::value);
}

}  // namespace
