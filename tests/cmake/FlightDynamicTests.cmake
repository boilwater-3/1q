# flight_dynamic CTest tiers. The executable is created by TestTargets.cmake.

if(NOT ONEQ_ENABLE_FLIGHT_DYNAMIC)
    return()
endif()

set(_fd_smoke_filter
    "FlightDynamicTest.AdapterCreatesAndRuns:"
    "FlightDynamicTest.VehicleStatePopulated:"
    "FlightDynamicTest.InitialVelocityDefaultsToBodyFrame:"
    "FlightDynamicTest.AutopilotSetsHeading:"
    "FlightDynamicTest.AutopilotSetsAltitude:"
    "FlightDynamicTest.WaypointManagerAddsWaypoints:"
    "FlightDynamicTest.FlyToWaypointManeuver:"
    "FlightDynamicTest.OrbitManeuver:"
    "FlightDynamicTest.AbortManeuver:"
    "FlightDynamicTest.ResetAndReuse:"
    "*BareAircraftTest.ModelLoadsSuccessfully/*:"
    "*ProfileSnapshotTest.MatchesExpectedProfile/*")
string(REPLACE ";" "" _fd_smoke_filter_str "${_fd_smoke_filter}")
add_test(NAME "fd_smoke::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_smoke_filter_str})
set_tests_properties("fd_smoke::1q_fd_tests" PROPERTIES LABELS "fd_smoke;fd_ci")

set(_fd_contract_filter
    "FlightDynamicTest.AutopilotDetectsOwnApProfile:"
    "FlightDynamicTest.AutopilotDetectsFbwProfile:"
    "FlightDynamicTest.AutopilotDetectsC310Profile:"
    "FlightDynamicTest.AutopilotDetectsConcordeProfile:"
    "FlightDynamicTest.AutopilotWritesDetectedYawInputProperty:"
    "FlightDynamicTest.InitialVelocityCanUseEcefFrame:"
    "FlightDynamicTest.InitialConditionsAcceptEcefPosition:"
    "*BareAircraftTest.ProjectIcTrimOnPassesL1/*:"
    "*BareAircraftTest.ProjectIcTrimOffPassesL1/*:"
    "*BareAircraftDiagnosticTest.*:"
    "*BareAircraftTest.TrimBehaviorRecorded/*:"
    "*ResetXmlBaselineTest.ResetXmlFreeFlightStable/*")
string(REPLACE ";" "" _fd_contract_filter_str "${_fd_contract_filter}")
add_test(NAME "fd_contract::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_contract_filter_str})
set_tests_properties("fd_contract::1q_fd_tests" PROPERTIES LABELS "fd_contract;fd_ci")

set(_fd_ctrl_filter
    "FlightDynamicTest.MultipleManeuvers:"
    "FlightDynamicTest.SetPitchManeuver:"
    "FlightDynamicTest.SetRollManeuver:"
    "FlightDynamicTest.SetAltitudeManeuver:"
    "FlightDynamicRobustnessTest.*:"
    "*BareAircraftTest.ProjectIcTrimOnFreeFlightStable/*:"
    "*BareAircraftTest.ProjectIcTrimOffFreeFlightStable/*")
string(REPLACE ";" "" _fd_ctrl_filter_str "${_fd_ctrl_filter}")
add_test(NAME "fd_controllability::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_ctrl_filter_str})
set_tests_properties("fd_controllability::1q_fd_tests" PROPERTIES LABELS "fd_controllability;fd_ci")

set(_fd_perf_filter
    "*AircraftManeuverTest.FlyToWaypoint/*:"
    "*AircraftManeuverTest.SetAltitudeClimb/*:"
    "*AircraftManeuverTest.SetHeading/*:"
    "*AircraftManeuverTest.OrbitTimedCompletion/*:"
    "*AircraftManeuverTest.QueueOrbitThenHeading/*:"
    "*AircraftManeuverTest.QueueFlyToThenOrbit/*")
string(REPLACE ";" "" _fd_perf_filter_str "${_fd_perf_filter}")
string(APPEND _fd_perf_filter_str ":-*KnownLimitFighterModels*")
add_test(NAME "fd_performance::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_perf_filter_str})
set_tests_properties("fd_performance::1q_fd_tests" PROPERTIES LABELS "fd_performance")

set(_fd_limit_filter
    "*AircraftManeuverTest.SetRoll/*:"
    "*AircraftManeuverTest.ResetAndReuse/*:"
    "*AircraftManeuverTest.AbortManeuver/*:"
    "*AircraftManeuverTest.InvalidOrbitRadius/*:"
    "*OrbitUnitTest.*/*:"
    "*NewManeuverSmoke.*"
    "*FdAircraftProbe.*")
string(REPLACE ";" "" _fd_limit_filter_str "${_fd_limit_filter}")
add_test(NAME "fd_known_limit::1q_fd_tests"
         COMMAND ${PROJECT_NAME}_fd_tests --gtest_filter=${_fd_limit_filter_str})
set_tests_properties("fd_known_limit::1q_fd_tests" PROPERTIES LABELS "fd_known_limit")

if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
    set(FD_JSBSIM_ROOT_DIR "${ONEQ_JSBSIM_DATA_ROOT_DIR}")
else()
    set(FD_JSBSIM_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
endif()
target_include_directories(${PROJECT_NAME}_fd_tests SYSTEM PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/jsbsim/src)
if(TARGET JSBSim::JSBSim)
    target_link_libraries(${PROJECT_NAME}_fd_tests PRIVATE JSBSim::JSBSim)
endif()
target_compile_definitions(${PROJECT_NAME}_fd_tests PRIVATE
    FD_JSBSIM_ROOT_DIR="${FD_JSBSIM_ROOT_DIR}")
