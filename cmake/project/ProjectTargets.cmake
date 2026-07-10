# Product target helpers. They deliberately standardize only stable build
# mechanics; source ownership and direct dependencies remain visible in modules.

function(oneq_apply_target_configuration)
    set(multi_value_args TARGETS LINK_TARGETS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "oneq_apply_target_configuration() requires TARGETS")
    endif()
    if(NOT ARG_LINK_TARGETS)
        set(ARG_LINK_TARGETS ${ARG_TARGETS})
    endif()

    apply_unity_build(TARGETS ${ARG_TARGETS})
    if(MSVC)
        apply_msvc_options(
            TARGETS ${ARG_TARGETS}
            LINK_TARGETS ${ARG_LINK_TARGETS}
            ENABLE_WARNINGS ${ENABLE_WARNINGS}
            STACK_SIZE_OPTION ${STACK_SIZE_OPTION})
    else()
        apply_clang_gcc_options(
            TARGETS ${ARG_TARGETS}
            LINK_TARGETS ${ARG_LINK_TARGETS}
            ENABLE_WARNINGS ${ENABLE_WARNINGS}
            STACK_SIZE_OPTION ${STACK_SIZE_OPTION})
    endif()
    apply_coverage_options(
        TARGETS ${ARG_TARGETS}
        LINK_TARGETS ${ARG_LINK_TARGETS})
    apply_precompiled_headers(TARGETS ${ARG_TARGETS})
endfunction()

function(oneq_configure_public_library target)
    if(NOT TARGET "${target}")
        message(FATAL_ERROR "oneq_configure_public_library(): target does not exist: ${target}")
    endif()

    set(ONEQ_MSVC_RUNTIME_LIBRARY "" CACHE STRING
        "MSVC runtime override. Empty = default dynamic CRT (/MD /MDd).")
    if(MSVC)
        if(ONEQ_MSVC_RUNTIME_LIBRARY STREQUAL "")
            set(_oneq_msvc_rt "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
        else()
            set(_oneq_msvc_rt "${ONEQ_MSVC_RUNTIME_LIBRARY}")
        endif()
        set_property(TARGET "${target}" PROPERTY MSVC_RUNTIME_LIBRARY "${_oneq_msvc_rt}")
    endif()

    if(NOT BUILD_SHARED_LIBS)
        target_compile_definitions("${target}" PUBLIC ONEQ_STATIC_DEFINE)
    endif()

    set_target_properties("${target}" PROPERTIES
        EXPORT_NAME "${PROJECT_NAME_LOWER}"
        OUTPUT_NAME "${PROJECT_NAME_LOWER}"
        DEBUG_POSTFIX d
        VERSION "${PROJECT_VERSION}"
        SOVERSION "${PROJECT_VERSION_MAJOR}")
    if(NOT WIN32)
        set_target_properties("${target}" PROPERTIES
            CXX_VISIBILITY_PRESET hidden
            VISIBILITY_INLINES_HIDDEN YES)
    endif()
endfunction()

function(oneq_add_component target)
    set(multi_value_args SOURCES)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})
    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "oneq_add_component(${target}) requires SOURCES")
    endif()
    if(TARGET "${target}")
        message(FATAL_ERROR "1q component target already exists: ${target}")
    endif()

    set(component_sources)
    foreach(component_source IN LISTS ARG_SOURCES)
        if(IS_ABSOLUTE "${component_source}")
            list(APPEND component_sources "${component_source}")
        else()
            list(APPEND component_sources
                "${CMAKE_CURRENT_SOURCE_DIR}/${component_source}")
        endif()
    endforeach()

    add_library("${target}" OBJECT ${component_sources})
    target_include_directories("${target}" PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${PROJECT_GENERATED_INCLUDE_DIR}"
        "${CMAKE_SOURCE_DIR}/include"
        "${ONEQ_FLATBUFFERS_GENERATED_DIR}")
    target_compile_features("${target}" PRIVATE "cxx_std_${PROJECT_CXX_STANDARD}")
    if(NOT BUILD_SHARED_LIBS)
        target_compile_definitions("${target}" PRIVATE ONEQ_STATIC_DEFINE)
    endif()
    if(PROJECT_ENABLE_SPDLOG)
        target_link_libraries("${target}" PRIVATE "${PROJECT_SPDLOG_TARGET}")
    endif()
    target_compile_definitions("${target}" PRIVATE
        PROJECT_LOG_BACKEND_SPDLOG=$<BOOL:${PROJECT_ENABLE_SPDLOG}>
        ONEQ_HAVE_ZLIB=$<BOOL:${ONEQ_HAVE_ZLIB}>)
    if(ONEQ_HAVE_ZLIB)
        target_link_libraries("${target}" PRIVATE ZLIB::ZLIB)
    endif()
    oneq_apply_target_configuration(TARGETS "${target}")
    set_property(GLOBAL APPEND PROPERTY ONEQ_COMPONENT_TARGETS "${target}")
endfunction()

function(oneq_get_component_targets out_var)
    get_property(component_targets GLOBAL PROPERTY ONEQ_COMPONENT_TARGETS)
    set("${out_var}" "${component_targets}" PARENT_SCOPE)
endfunction()
