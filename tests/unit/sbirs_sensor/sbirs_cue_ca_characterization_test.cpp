#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <vector>

namespace sbirs_sensor {
namespace pipeline {
namespace {

constexpr double kNumericTolerance = 1.0e-9;

double NormalizeAzimuth(double azimuth_deg) {
  double result = std::fmod(azimuth_deg + 180.0, 360.0);
  if (result < 0.0) {
    result += 360.0;
  }
  return result - 180.0;
}

double AngularError(double predicted_deg, double truth_deg) {
  return std::fabs(NormalizeAzimuth(predicted_deg - truth_deg));
}

struct CueSample {
  double time_sec{0.0};
  double unwrapped_azimuth_deg{0.0};
};

class CharacterizationCuePredictor {
 public:
  double UpdateCv(double measured_azimuth_deg, double dt_sec, double latency_sec) {
    const double normalized = NormalizeAzimuth(measured_azimuth_deg);
    if (!has_previous_ || !std::isfinite(dt_sec) || dt_sec <= 0.0 || latency_sec <= 0.0) {
      previous_azimuth_deg_ = normalized;
      has_previous_ = true;
      return normalized;
    }
    const double rate_deg_per_sec = NormalizeAzimuth(normalized - previous_azimuth_deg_) / dt_sec;
    previous_azimuth_deg_ = normalized;
    return NormalizeAzimuth(normalized + rate_deg_per_sec * latency_sec);
  }

  double UpdateCa(double measured_azimuth_deg, double dt_sec, double latency_sec) {
    const double normalized = NormalizeAzimuth(measured_azimuth_deg);
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
      samples_.clear();
      elapsed_sec_ = 0.0;
    } else if (!samples_.empty()) {
      elapsed_sec_ += dt_sec;
    }

    double unwrapped = normalized;
    if (!samples_.empty()) {
      const double previous_normalized = NormalizeAzimuth(samples_.back().unwrapped_azimuth_deg);
      unwrapped = samples_.back().unwrapped_azimuth_deg +
                  NormalizeAzimuth(normalized - previous_normalized);
    }
    samples_.push_back({elapsed_sec_, unwrapped});
    while (samples_.size() > 5U) {
      samples_.pop_front();
    }

    if (latency_sec <= 0.0 || samples_.size() == 1U) {
      return normalized;
    }
    if (samples_.size() == 2U) {
      const CueSample& previous = samples_[0];
      const CueSample& current = samples_[1];
      const double sample_dt = current.time_sec - previous.time_sec;
      if (sample_dt <= 0.0) {
        return normalized;
      }
      const double rate =
          (current.unwrapped_azimuth_deg - previous.unwrapped_azimuth_deg) / sample_dt;
      return NormalizeAzimuth(current.unwrapped_azimuth_deg + rate * latency_sec);
    }

    std::array<std::array<double, 4>, 3> system{};
    const double current_time = samples_.back().time_sec;
    for (const CueSample& sample : samples_) {
      const double t = sample.time_sec - current_time;
      const std::array<double, 3> basis{{1.0, t, 0.5 * t * t}};
      for (std::size_t row = 0; row < 3U; ++row) {
        for (std::size_t column = 0; column < 3U; ++column) {
          system[row][column] += basis[row] * basis[column];
        }
        system[row][3] += basis[row] * sample.unwrapped_azimuth_deg;
      }
    }
    if (!Solve(&system)) {
      return normalized;
    }
    const double command =
        system[0][3] + system[1][3] * latency_sec + 0.5 * system[2][3] * latency_sec * latency_sec;
    return NormalizeAzimuth(command);
  }

 private:
  static bool Solve(std::array<std::array<double, 4>, 3>* system) {
    for (std::size_t column = 0; column < 3U; ++column) {
      std::size_t pivot = column;
      for (std::size_t row = column + 1U; row < 3U; ++row) {
        if (std::fabs((*system)[row][column]) > std::fabs((*system)[pivot][column])) {
          pivot = row;
        }
      }
      if (std::fabs((*system)[pivot][column]) < 1.0e-12) {
        return false;
      }
      std::swap((*system)[column], (*system)[pivot]);
      const double divisor = (*system)[column][column];
      for (std::size_t value = column; value < 4U; ++value) {
        (*system)[column][value] /= divisor;
      }
      for (std::size_t row = 0; row < 3U; ++row) {
        if (row == column) {
          continue;
        }
        const double factor = (*system)[row][column];
        for (std::size_t value = column; value < 4U; ++value) {
          (*system)[row][value] -= factor * (*system)[column][value];
        }
      }
    }
    return true;
  }

  bool has_previous_{false};
  double previous_azimuth_deg_{0.0};
  double elapsed_sec_{0.0};
  std::deque<CueSample> samples_{};
};

struct Scenario {
  double initial_azimuth_deg{0.0};
  double angular_rate_deg_per_sec{0.0};
  double angular_acceleration_deg_per_sec2{0.0};
  double dt_sec{0.1};
  double latency_sec{0.2};
  double measurement_sigma_deg{0.0};
  double dt_variation_fraction{0.0};
  double acceleration_reversal_time_sec{-1.0};
  std::uint32_t random_seed{7U};
  double capture_half_width_deg{0.08};
};

struct Metrics {
  double rms_error_deg{0.0};
  double p95_error_deg{0.0};
  double capture_rate{0.0};
  bool all_finite{true};
};

double TruthAzimuth(const Scenario& scenario, double time_sec) {
  if (scenario.acceleration_reversal_time_sec >= 0.0 &&
      time_sec > scenario.acceleration_reversal_time_sec) {
    const double reversal_time = scenario.acceleration_reversal_time_sec;
    const double reversal_azimuth =
        scenario.initial_azimuth_deg + scenario.angular_rate_deg_per_sec * reversal_time +
        0.5 * scenario.angular_acceleration_deg_per_sec2 * reversal_time * reversal_time;
    const double reversal_rate = scenario.angular_rate_deg_per_sec +
                                 scenario.angular_acceleration_deg_per_sec2 * reversal_time;
    const double elapsed = time_sec - reversal_time;
    return NormalizeAzimuth(reversal_azimuth + reversal_rate * elapsed -
                            0.5 * scenario.angular_acceleration_deg_per_sec2 * elapsed * elapsed);
  }
  return NormalizeAzimuth(scenario.initial_azimuth_deg +
                          scenario.angular_rate_deg_per_sec * time_sec +
                          0.5 * scenario.angular_acceleration_deg_per_sec2 * time_sec * time_sec);
}

std::array<Metrics, 2> Evaluate(const Scenario& scenario) {
  CharacterizationCuePredictor cv;
  CharacterizationCuePredictor ca;
  std::mt19937 random_engine(scenario.random_seed);
  // sigma=0（无噪声场景）下 MSVC Debug CRT 断言 normal_distribution 构造非法；
  // 无噪声时退化为恒零采样器，保持数值行为不变。
  const bool has_noise = scenario.measurement_sigma_deg > 0.0;
  std::normal_distribution<double> noise(0.0,
                                         has_noise ? scenario.measurement_sigma_deg : 1.0);
  std::array<std::vector<double>, 2> errors;
  std::array<std::size_t, 2> captures{{0U, 0U}};
  std::array<bool, 2> all_finite{{true, true}};

  double time_sec = 0.0;
  for (std::size_t cycle = 0; cycle < 96U; ++cycle) {
    const double variation_sign = cycle % 2U == 0U ? -1.0 : 1.0;
    const double current_dt =
        scenario.dt_sec * (1.0 + variation_sign * scenario.dt_variation_fraction);
    if (cycle > 0U) {
      time_sec += current_dt;
    }
    const double measured =
        NormalizeAzimuth(TruthAzimuth(scenario, time_sec) +
                         (has_noise ? noise(random_engine) : 0.0));
    const std::array<double, 2> commands{{
        cv.UpdateCv(measured, current_dt, scenario.latency_sec),
        ca.UpdateCa(measured, current_dt, scenario.latency_sec),
    }};
    if (cycle < 4U) {
      continue;
    }
    const double future_truth = TruthAzimuth(scenario, time_sec + scenario.latency_sec);
    for (std::size_t model = 0; model < 2U; ++model) {
      all_finite[model] = all_finite[model] && std::isfinite(commands[model]);
      const double error = AngularError(commands[model], future_truth);
      errors[model].push_back(error);
      if (error <= scenario.capture_half_width_deg) {
        ++captures[model];
      }
    }
  }

  std::array<Metrics, 2> metrics;
  for (std::size_t model = 0; model < 2U; ++model) {
    double squared_error_sum = 0.0;
    for (const double error : errors[model]) {
      squared_error_sum += error * error;
    }
    std::sort(errors[model].begin(), errors[model].end());
    const std::size_t p95_index =
        static_cast<std::size_t>(0.95 * static_cast<double>(errors[model].size() - 1U));
    metrics[model].rms_error_deg =
        std::sqrt(squared_error_sum / static_cast<double>(errors[model].size()));
    metrics[model].p95_error_deg = errors[model][p95_index];
    metrics[model].capture_rate =
        static_cast<double>(captures[model]) / static_cast<double>(errors[model].size());
    metrics[model].all_finite = all_finite[model];
  }
  return metrics;
}

TEST(SbirsCueCaCharacterizationTest, SustainedAccelerationPassesBenefitGateWithoutNoise) {
  const std::array<double, 3> sample_periods{{0.05, 0.1, 0.25}};
  const std::array<double, 3> latencies{{0.1, 0.2, 0.5}};
  const std::array<double, 3> rates{{-2.0, 0.0, 2.0}};
  const std::array<double, 4> accelerations{{-1.0, -0.25, 0.25, 1.0}};
  double cv_squared_sum = 0.0;
  double ca_squared_sum = 0.0;
  std::size_t scenario_count = 0U;
  for (const double dt : sample_periods) {
    for (const double latency : latencies) {
      for (const double rate : rates) {
        for (const double acceleration : accelerations) {
          Scenario scenario;
          scenario.dt_sec = dt;
          scenario.latency_sec = latency;
          scenario.angular_rate_deg_per_sec = rate;
          scenario.angular_acceleration_deg_per_sec2 = acceleration;
          const std::array<Metrics, 2> metrics = Evaluate(scenario);
          cv_squared_sum += metrics[0].rms_error_deg * metrics[0].rms_error_deg;
          ca_squared_sum += metrics[1].rms_error_deg * metrics[1].rms_error_deg;
          ++scenario_count;
          EXPECT_GE(metrics[1].capture_rate + kNumericTolerance, metrics[0].capture_rate);
        }
      }
    }
  }
  const double cv_rms = std::sqrt(cv_squared_sum / static_cast<double>(scenario_count));
  const double ca_rms = std::sqrt(ca_squared_sum / static_cast<double>(scenario_count));
  RecordProperty("scenario_count", static_cast<int>(scenario_count));
  RecordProperty("cv_rms_deg", std::to_string(cv_rms));
  RecordProperty("ca_rms_deg", std::to_string(ca_rms));
  EXPECT_LE(ca_rms, 0.75 * cv_rms);
}

TEST(SbirsCueCaCharacterizationTest, StaticAndConstantVelocityHaveNoNoiselessRegression) {
  const std::array<double, 3> rates{{-2.0, 0.0, 2.0}};
  const std::array<double, 3> sample_periods{{0.05, 0.1, 0.25}};
  const std::array<double, 3> latencies{{0.1, 0.2, 0.5}};
  for (const double rate : rates) {
    for (const double dt : sample_periods) {
      for (const double latency : latencies) {
        Scenario scenario;
        scenario.angular_rate_deg_per_sec = rate;
        scenario.dt_sec = dt;
        scenario.latency_sec = latency;
        const std::array<Metrics, 2> metrics = Evaluate(scenario);
        EXPECT_LE(metrics[1].rms_error_deg, metrics[0].rms_error_deg + kNumericTolerance);
        EXPECT_LE(metrics[1].p95_error_deg, metrics[0].p95_error_deg + kNumericTolerance);
        EXPECT_GE(metrics[1].capture_rate + kNumericTolerance, metrics[0].capture_rate);
      }
    }
  }
}

TEST(SbirsCueCaCharacterizationTest, FiveSampleCaFailsStrictNominalNoiseZeroRegression) {
  Scenario scenario;
  scenario.angular_rate_deg_per_sec = 1.0;
  scenario.latency_sec = 0.5;
  scenario.measurement_sigma_deg = 0.01;
  const std::array<Metrics, 2> metrics = Evaluate(scenario);

  EXPECT_TRUE(metrics[0].all_finite);
  EXPECT_TRUE(metrics[1].all_finite);
  RecordProperty("cv_rms_deg", std::to_string(metrics[0].rms_error_deg));
  RecordProperty("ca_rms_deg", std::to_string(metrics[1].rms_error_deg));
  RecordProperty("cv_p95_deg", std::to_string(metrics[0].p95_error_deg));
  RecordProperty("ca_p95_deg", std::to_string(metrics[1].p95_error_deg));
  RecordProperty("cv_capture_rate", std::to_string(metrics[0].capture_rate));
  RecordProperty("ca_capture_rate", std::to_string(metrics[1].capture_rate));
  EXPECT_GT(metrics[1].rms_error_deg, metrics[0].rms_error_deg + kNumericTolerance);
  EXPECT_GT(metrics[1].p95_error_deg, metrics[0].p95_error_deg + kNumericTolerance);
  EXPECT_LT(metrics[1].capture_rate, metrics[0].capture_rate);
}

TEST(SbirsCueCaCharacterizationTest, HighNoiseAndAzimuthWrapRemainFinite) {
  Scenario scenario;
  scenario.initial_azimuth_deg = 179.0;
  scenario.angular_rate_deg_per_sec = 2.0;
  scenario.angular_acceleration_deg_per_sec2 = 0.5;
  scenario.measurement_sigma_deg = 0.1;
  const std::array<Metrics, 2> metrics = Evaluate(scenario);
  EXPECT_TRUE(metrics[0].all_finite);
  EXPECT_TRUE(metrics[1].all_finite);
}

TEST(SbirsCueCaCharacterizationTest, NonUniformDtAndAccelerationReversalRemainFinite) {
  Scenario scenario;
  scenario.initial_azimuth_deg = -179.0;
  scenario.angular_rate_deg_per_sec = -1.0;
  scenario.angular_acceleration_deg_per_sec2 = 0.75;
  scenario.dt_variation_fraction = 0.2;
  scenario.acceleration_reversal_time_sec = 3.0;
  scenario.measurement_sigma_deg = 0.01;
  const std::array<Metrics, 2> metrics = Evaluate(scenario);
  EXPECT_TRUE(metrics[0].all_finite);
  EXPECT_TRUE(metrics[1].all_finite);
}

}  // namespace
}  // namespace pipeline
}  // namespace sbirs_sensor
