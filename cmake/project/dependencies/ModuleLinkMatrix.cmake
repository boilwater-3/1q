# Applies resolved package targets to 1q build targets.

target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE ${ONEQ_LINK_DEPENDENCIES})

target_link_libraries(airborne_engine PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(airborne_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(esr_engine PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(esr_core PRIVATE ${ONEQ_LINK_DEPENDENCIES})
target_link_libraries(eos_engine PRIVATE flatbuffers::flatbuffers)
target_link_libraries(eos_core PRIVATE flatbuffers::flatbuffers)
target_link_libraries(sbirs_engine PRIVATE Eigen3::Eigen flatbuffers::flatbuffers)
target_link_libraries(sbirs_core PRIVATE Eigen3::Eigen flatbuffers::flatbuffers)
target_link_libraries(sar_core PRIVATE flatbuffers::flatbuffers)

if(ONEQ_ENABLE_HDF5_OUTPUT)
    find_package(HighFive CONFIG REQUIRED)
    message(STATUS "SAR HDF5 output: ENABLED (HighFive found)")
    if(TARGET sar_engine)
        target_link_libraries(sar_engine PRIVATE HighFive::HighFive)
        target_compile_definitions(sar_engine PRIVATE ONEQ_ENABLE_HDF5_OUTPUT)
    endif()
else()
    message(STATUS "SAR HDF5 output: disabled (ONEQ_ENABLE_HDF5_OUTPUT=OFF)")
endif()

if(TARGET sar_engine)
    target_link_libraries(sar_engine PRIVATE Eigen3::Eigen)
endif()

if(TARGET fd_engine)
    target_link_libraries(fd_engine PRIVATE JSBSim::JSBSim)
endif()
if(TARGET fd_core)
    target_link_libraries(fd_core PRIVATE JSBSim::JSBSim)
endif()
target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE JSBSim::JSBSim)

set(_oneq_conan_include_vars
    eigen_INCLUDE_DIRS_RELEASE
    boost_INCLUDE_DIRS_RELEASE
    nanoflann_INCLUDE_DIRS_RELEASE
    flatbuffers_INCLUDE_DIRS_RELEASE
    zlib_INCLUDE_DIRS_RELEASE)
foreach(_oneq_conan_include_var IN LISTS _oneq_conan_include_vars)
    if(DEFINED ${_oneq_conan_include_var})
        foreach(ONEQ_BUILD_TARGET IN ITEMS
            ${PROJECT_CORE_TARGET}
            ${ONEQ_OBJECT_TARGETS}
        )
            target_include_directories(${ONEQ_BUILD_TARGET} PRIVATE ${${_oneq_conan_include_var}})
        endforeach()
    endif()
endforeach()
unset(_oneq_conan_include_var)
unset(_oneq_conan_include_vars)
unset(ONEQ_BUILD_TARGET)

if(PROJECT_ENABLE_SPDLOG)
    if(TARGET spdlog::spdlog)
        set(PROJECT_SPDLOG_TARGET spdlog::spdlog)
    elseif(TARGET spdlog::spdlog_header_only)
        message(FATAL_ERROR "spdlog header-only target is not allowed in this configuration")
    else()
        message(FATAL_ERROR "spdlog target is not available")
    endif()
    foreach(ONEQ_BUILD_TARGET IN ITEMS
        ${PROJECT_CORE_TARGET}
        ${ONEQ_OBJECT_TARGETS}
    )
        target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ${PROJECT_SPDLOG_TARGET})
    endforeach()
    unset(ONEQ_BUILD_TARGET)
endif()

foreach(ONEQ_BUILD_TARGET IN ITEMS
    ${PROJECT_CORE_TARGET}
    ${ONEQ_OBJECT_TARGETS}
)
    target_compile_definitions(${ONEQ_BUILD_TARGET}
        PRIVATE PROJECT_LOG_BACKEND_SPDLOG=$<BOOL:${PROJECT_ENABLE_SPDLOG}>
                ONEQ_HAVE_ZLIB=$<BOOL:${ONEQ_HAVE_ZLIB}>
    )
endforeach()
unset(ONEQ_BUILD_TARGET)

if(ONEQ_HAVE_ZLIB)
    foreach(ONEQ_BUILD_TARGET IN ITEMS
        ${PROJECT_CORE_TARGET}
        ${ONEQ_OBJECT_TARGETS}
    )
        target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ZLIB::ZLIB)
    endforeach()
    unset(ONEQ_BUILD_TARGET)
endif()
