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
