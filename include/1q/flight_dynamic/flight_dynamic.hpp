/**
 * @file flight_dynamic.hpp
 * @brief flight_dynamic 模块 umbrella header — 单文件引入所有公共 API。
 *
 * 推荐使用方式：
 * @code
 * #include "1q/flight_dynamic/flight_dynamic.hpp"
 * @endcode
 */

#ifndef FLIGHT_DYNAMIC_FLIGHT_DYNAMIC_HPP_
#define FLIGHT_DYNAMIC_FLIGHT_DYNAMIC_HPP_

// 配置
#include "1q/flight_dynamic/config/AircraftDefinition.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"

// 数据模型
#include "1q/flight_dynamic/model/VehicleState.h"
#include "1q/flight_dynamic/model/FlightDynamicInput.h"
#include "1q/flight_dynamic/model/FlightDynamicOutput.h"

// 会话与工厂
#include "1q/flight_dynamic/session/FlightDynamicSession.h"
#include "1q/flight_dynamic/session/FlightDynamicSessionFactory.h"

#endif  // FLIGHT_DYNAMIC_FLIGHT_DYNAMIC_HPP_
