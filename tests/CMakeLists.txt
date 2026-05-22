if(NOT ENABLE_TESTING)
    return()
endif()

if(MSVC AND MSVC_VERSION LESS 1910)
    set(CMAKE_MAP_IMPORTED_CONFIG_DEBUG Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_MINSIZEREL Release)
endif()

if(PACKAGE_MANAGER STREQUAL "conan" OR PACKAGE_MANAGER STREQUAL "vcpkg")
    find_package(GTest CONFIG REQUIRED)
    find_package(Boost CONFIG REQUIRED)
    find_package(Eigen3 CONFIG REQUIRED)
    find_package(flatbuffers CONFIG REQUIRED)
    find_package(ZLIB QUIET)
endif()
if(NOT DEFINED flatbuffers_INCLUDE_DIRS_RELEASE)
    # Pull Conan-generated include variables without importing flatc target.
    file(GLOB _oneq_flatbuffers_data_candidates
        "${CMAKE_BINARY_DIR}/build/generators/flatbuffers-*-data.cmake")
    foreach(_oneq_flatbuffers_data IN LISTS _oneq_flatbuffers_data_candidates)
        if(NOT _oneq_flatbuffers_data MATCHES "module-flatbuffers")
            include("${_oneq_flatbuffers_data}")
            break()
        endif()
    endforeach()
    unset(_oneq_flatbuffers_data)
    unset(_oneq_flatbuffers_data_candidates)
endif()

file(GLOB_RECURSE UNIT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp"
)
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp"
)
file(GLOB_RECURSE CONTRACT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/contract/*.cpp"
)

set(TEST_SOURCES
    ${UNIT_TEST_SOURCES}
    ${INTEGRATION_TEST_SOURCES}
    ${CONTRACT_TEST_SOURCES}
)

if(NOT TEST_SOURCES)
    message(STATUS "No test sources defined in ${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "  Add .cpp files to unit/integration/contract to enable testing")
    return()
endif()

include(GoogleTest)
function(add_1q_gtest target_name suite_label discovery_timeout)
    if(NOT ARGN)
        return()
    endif()

    add_executable(${target_name} ${ARGN})
    target_link_libraries(${target_name} PRIVATE
        GTest::gtest_main
        GTest::gmock
        Boost::boost
        Eigen3::Eigen
        ${PROJECT_ALIAS}
    )
    if(TARGET ZLIB::ZLIB)
        target_link_libraries(${target_name} PRIVATE ZLIB::ZLIB)
    endif()
    if(TARGET flatbuffers::flatbuffers)
        target_link_libraries(${target_name} PRIVATE flatbuffers::flatbuffers)
    endif()
    target_include_directories(${target_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    if(DEFINED flatbuffers_INCLUDE_DIRS_RELEASE)
        # Conan multi-config can omit header-only include dirs for tests.
        # Mirror Release include dirs so flexbuffers headers are always resolvable.
        target_include_directories(${target_name} PRIVATE
            ${flatbuffers_INCLUDE_DIRS_RELEASE}
        )
    endif()
    if(WIN32)
        # flatbuffers/1.12 header-only mode on VS2015 needs this to avoid
        # unresolved ClassicLocale symbols from util.h.
        target_compile_definitions(${target_name} PRIVATE
            FLATBUFFERS_LOCALE_INDEPENDENT=0
        )
    endif()

    # Propagate ONEQ_HAVE_ZLIB so compression test guards resolve the same as in the library.
    if(TARGET ZLIB::ZLIB)
        target_compile_definitions(${target_name} PRIVATE ONEQ_HAVE_ZLIB=1)
        if(DEFINED zlib_INCLUDE_DIRS_RELEASE)
            target_include_directories(${target_name} PRIVATE ${zlib_INCLUDE_DIRS_RELEASE})
        endif()
    else()
        target_compile_definitions(${target_name} PRIVATE ONEQ_HAVE_ZLIB=0)
    endif()

    if(WIN32 AND BUILD_SHARED_LIBS)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${PROJECT_CORE_TARGET}>
                $<TARGET_FILE_DIR:${target_name}>
            COMMENT "Copying ${PROJECT_CORE_TARGET} DLL to test directory"
        )
    endif()

    gtest_discover_tests(${target_name}
        DISCOVERY_MODE POST_BUILD
        DISCOVERY_TIMEOUT ${discovery_timeout}
        TEST_PREFIX "${suite_label}::"
        PROPERTIES LABELS "${suite_label}"
    )
endfunction()

add_1q_gtest(${PROJECT_NAME}_unit_tests unit 60 ${UNIT_TEST_SOURCES})
add_1q_gtest(${PROJECT_NAME}_integration_tests integration 120 ${INTEGRATION_TEST_SOURCES})
add_1q_gtest(${PROJECT_NAME}_contract_tests contract 60 ${CONTRACT_TEST_SOURCES})

# flight_dynamic 测试需要 JSBSim 数据文件路径
set(FD_JSBSIM_ROOT_DIR "${CMAKE_SOURCE_DIR}/third_party/jsbsim")
foreach(_fd_test_target IN ITEMS ${PROJECT_NAME}_unit_tests ${PROJECT_NAME}_integration_tests)
  target_compile_definitions(${_fd_test_target}
      PRIVATE FD_JSBSIM_ROOT_DIR="${FD_JSBSIM_ROOT_DIR}")
  # 测试可执行文件需显式链接 JSBSim 共享库（静态主库不传递 PRIVATE 依赖）
  target_link_libraries(${_fd_test_target} PRIVATE JSBSim::JSBSim)
  # 将 JSBSim 共享库目录加入 rpath，避免设置 DYLD_LIBRARY_PATH
  set_property(TARGET ${_fd_test_target} APPEND PROPERTY
      BUILD_RPATH "$<TARGET_FILE_DIR:libJSBSim>")
endforeach()
unset(_fd_test_target)

set(REPLAY_FAST_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/replay_trace_writer_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/ar_trace_session_adapter_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/ar_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/eos_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/eos_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/esr_replay_codec_roundtrip_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/esr_replay_session_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit/replay_trace_compression_test.cpp
)
add_1q_gtest(${PROJECT_NAME}_replay_fast_tests replay_fast 90 ${REPLAY_FAST_TEST_SOURCES})

add_custom_target(${PROJECT_NAME}_tests)
add_dependencies(${PROJECT_NAME}_tests
    ${PROJECT_NAME}_unit_tests
    ${PROJECT_NAME}_integration_tests
    ${PROJECT_NAME}_contract_tests
)

add_custom_target(${PROJECT_NAME}_tests_fast)
add_dependencies(${PROJECT_NAME}_tests_fast
    ${PROJECT_NAME}_replay_fast_tests
)

add_test(
    NAME public_api_boundary_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_public_api_boundary.cmake
)

add_test(
    NAME airborne_include_style_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_airborne_include_style.cmake
)
add_test(
    NAME esr_include_style_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_esr_include_style.cmake
)

add_test(
    NAME eos_include_style_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_eos_include_style.cmake
)

add_test(
    NAME airborne_include_direction_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_airborne_include_direction.cmake
)
add_test(
    NAME esr_include_direction_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_esr_include_direction.cmake
)
add_test(
    NAME eos_include_direction_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_eos_include_direction.cmake
)
add_test(
    NAME public_header_external_dependency_isolation_guard
    COMMAND ${CMAKE_COMMAND}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/contract/check_public_header_external_dependency_isolation.cmake
)

set_tests_properties(
    public_api_boundary_guard
    airborne_include_style_guard
    esr_include_style_guard
    eos_include_style_guard
    airborne_include_direction_guard
    esr_include_direction_guard
    eos_include_direction_guard
    public_header_external_dependency_isolation_guard
    PROPERTIES LABELS "contract"
)

# Scene programs — standalone executables that exercise full module pipelines
file(GLOB SCENE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scene/*.cpp")
if(SCENE_SOURCES)
    foreach(_scene_src ${SCENE_SOURCES})
        get_filename_component(_scene_name ${_scene_src} NAME_WE)
        add_executable(${_scene_name} ${_scene_src})
        target_link_libraries(${_scene_name} PRIVATE
            ${PROJECT_ALIAS}
        )
        target_include_directories(${_scene_name} PRIVATE
            ${CMAKE_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/src
            ${CMAKE_CURRENT_SOURCE_DIR}
        )
        target_compile_definitions(${_scene_name} PRIVATE
            SCENE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scene/config"
        )
        # Scene programs include config_loader headers from the examples directory
        if(_scene_name MATCHES "airborne_radar")
            target_include_directories(${_scene_name} PRIVATE
                "${CMAKE_SOURCE_DIR}/examples/airborne_radar")
        elseif(_scene_name MATCHES "electro_optical")
            target_include_directories(${_scene_name} PRIVATE
                "${CMAKE_SOURCE_DIR}/examples/electro_optical")
        elseif(_scene_name MATCHES "electronic_warfare")
            target_include_directories(${_scene_name} PRIVATE
                "${CMAKE_SOURCE_DIR}/examples/electronic_warfare")
        endif()
    endforeach()
endif()
