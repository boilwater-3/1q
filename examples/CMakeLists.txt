# 示例程序构建入口

if(NOT TARGET ${PROJECT_ALIAS})
    message(FATAL_ERROR "${PROJECT_ALIAS} target not found. Ensure src/CMakeLists.txt is processed first")
endif()

function(add_domain_example name dir)
    add_executable(${name} "${dir}/session_usage.cpp")
    target_link_libraries(${name} PRIVATE ${PROJECT_ALIAS})
    target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
endfunction()

add_domain_example(airborne_radar_session_usage airborne_radar)
add_domain_example(electro_optical_sensor_session_usage electro_optical)
add_domain_example(electronic_surveillance_radar_session_usage electronic_warfare)
add_executable(ar_config_compare_test airborne_radar/config_compare_test.cpp)
target_link_libraries(ar_config_compare_test PRIVATE ${PROJECT_ALIAS})
target_include_directories(ar_config_compare_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar")

add_executable(takeoff_land_csv flight_dynamic/takeoff_land_csv.cpp)
target_link_libraries(takeoff_land_csv PRIVATE ${PROJECT_ALIAS})
target_include_directories(takeoff_land_csv PRIVATE
    "${CMAKE_SOURCE_DIR}/src")
target_include_directories(takeoff_land_csv SYSTEM PRIVATE
    "${CMAKE_SOURCE_DIR}/third_party/jsbsim/src")
if(DEFINED ONEQ_JSBSIM_DATA_ROOT_DIR AND NOT ONEQ_JSBSIM_DATA_ROOT_DIR STREQUAL "")
    set(FD_JSBSIM_ROOT_DIR "${ONEQ_JSBSIM_DATA_ROOT_DIR}")
else()
    set(FD_JSBSIM_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
endif()
target_compile_definitions(takeoff_land_csv PRIVATE
    FD_JSBSIM_ROOT_DIR="${FD_JSBSIM_ROOT_DIR}")
