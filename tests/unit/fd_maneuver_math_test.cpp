/**
 * @file fd_maneuver_math_test.cpp
 * @brief 几何与数学工具单元测试
 */

#include <gtest/gtest.h>

#include <cmath>

#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/maneuver/ManeuverController.h"

namespace fd_maneuver = flight_dynamic::maneuver;
namespace coord = oneq::coordinate;

TEST(FdManeuverMathTest, DistanceSamePoint) {
  coord::LlaPositionDegM p1{};
  p1.latitude_deg = 39.9;
  p1.longitude_deg = 116.4;
  p1.altitude_m = 1000.0;
  EXPECT_NEAR(fd_maneuver::ComputeGreatCircleDistanceM(p1, p1), 0.0, 1e-3);
}

TEST(FdManeuverMathTest, DistanceEquator) {
  coord::LlaPositionDegM p1{};
  coord::LlaPositionDegM p2{};
  p2.longitude_deg = 1.0;
  // 1 degree on equator with R=6371km is approx 111.1949 km
  EXPECT_NEAR(fd_maneuver::ComputeGreatCircleDistanceM(p1, p2), 111194.9, 10.0);
}

TEST(FdManeuverMathTest, DistancePoles) {
  coord::LlaPositionDegM n_pole{};
  n_pole.latitude_deg = 90.0;
  coord::LlaPositionDegM s_pole{};
  s_pole.latitude_deg = -90.0;
  // Pole to pole is half circumference: pi * R (~ 20015 km)
  EXPECT_NEAR(fd_maneuver::ComputeGreatCircleDistanceM(n_pole, s_pole), 20015000.0, 20000.0);
}

TEST(FdManeuverMathTest, AzimuthNorthEastSouthWest) {
  coord::LlaPositionDegM center{};
  coord::LlaPositionDegM north{};
  north.latitude_deg = 1.0;
  coord::LlaPositionDegM south{};
  south.latitude_deg = -1.0;
  coord::LlaPositionDegM east{};
  east.longitude_deg = 1.0;
  coord::LlaPositionDegM west{};
  west.longitude_deg = -1.0;

  EXPECT_NEAR(fd_maneuver::ComputeForwardAzimuthDeg(center, north), 0.0, 1e-3);
  EXPECT_NEAR(fd_maneuver::ComputeForwardAzimuthDeg(center, east), 90.0, 1e-3);
  EXPECT_NEAR(fd_maneuver::ComputeForwardAzimuthDeg(center, south), 180.0, 1e-3);
  EXPECT_NEAR(fd_maneuver::ComputeForwardAzimuthDeg(center, west), 270.0, 1e-3);
}

TEST(FdManeuverMathTest, AzimuthAntimeridian) {
  coord::LlaPositionDegM p1{};
  p1.longitude_deg = 179.9;
  coord::LlaPositionDegM p2{};
  p2.longitude_deg = -179.9;  // crossing the antimeridian eastwards
  EXPECT_NEAR(fd_maneuver::ComputeForwardAzimuthDeg(p1, p2), 90.0, 1e-3);
  EXPECT_NEAR(fd_maneuver::ComputeForwardAzimuthDeg(p2, p1), 270.0, 1e-3);
}

TEST(FdManeuverMathTest, NormalizeHeadingError) {
  EXPECT_NEAR(fd_maneuver::NormalizeHeadingErrorDeg(0.0), 0.0, 1e-5);
  EXPECT_NEAR(fd_maneuver::NormalizeHeadingErrorDeg(180.0), 180.0, 1e-5);
  EXPECT_NEAR(fd_maneuver::NormalizeHeadingErrorDeg(-180.0), -180.0, 1e-5);
  EXPECT_NEAR(fd_maneuver::NormalizeHeadingErrorDeg(181.0), -179.0, 1e-5);
  EXPECT_NEAR(fd_maneuver::NormalizeHeadingErrorDeg(-181.0), 179.0, 1e-5);
  EXPECT_NEAR(fd_maneuver::NormalizeHeadingErrorDeg(360.0), 0.0, 1e-5);
  EXPECT_NEAR(fd_maneuver::NormalizeHeadingErrorDeg(725.0), 5.0, 1e-5);
  EXPECT_NEAR(fd_maneuver::NormalizeHeadingErrorDeg(-725.0), -5.0, 1e-5);
}
