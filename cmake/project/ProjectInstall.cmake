# Installation and package-export lifecycle entry point.

function(oneq_configure_package_dependencies out_var)
    set(config_find_dependencies "")
    if(PACKAGE_MANAGER STREQUAL "conan")
        string(APPEND config_find_dependencies
            "find_dependency(Eigen3 REQUIRED CONFIG)\n"
            "find_dependency(Boost REQUIRED CONFIG)\n"
            "find_dependency(nanoflann REQUIRED CONFIG)\n"
            "find_dependency(flatbuffers REQUIRED CONFIG)\n"
            "find_dependency(jsbsim REQUIRED CONFIG)\n")
        if(PROJECT_ENABLE_SPDLOG)
            string(APPEND config_find_dependencies
                "find_dependency(spdlog REQUIRED CONFIG)\n")
        endif()
        string(APPEND config_find_dependencies "find_dependency(ZLIB REQUIRED)\n")
        if(ONEQ_ENABLE_HDF5_OUTPUT)
            string(APPEND config_find_dependencies
                "find_dependency(HighFive REQUIRED CONFIG)\n")
        endif()
    endif()
    set("${out_var}" "${config_find_dependencies}" PARENT_SCOPE)
endfunction()

function(oneq_install_project)
    if(NOT ENABLE_INSTALL)
        return()
    endif()

    include(CMakePackageConfigHelpers)

    install(TARGETS ${PROJECT_CORE_TARGET}
        EXPORT ${PROJECT_EXPORT_SET}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # include/1q/ is the guarded public-header boundary. Mirror it instead of
    # maintaining a second, module-local header manifest.
    install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/1q/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/1q"
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
            PATTERN "README.md")
    install(FILES "${PROJECT_GENERATED_EXPORT_HEADER}"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q
    )

    install(EXPORT ${PROJECT_EXPORT_SET}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
        FILE ${PROJECT_NAME}Targets.cmake
    )

    oneq_configure_package_dependencies(ONEQ_CONFIG_FIND_DEPENDENCIES)
    set(_oneq_config_template "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake.in")
    file(WRITE "${_oneq_config_template}" [=[@PACKAGE_INIT@

include(CMakeFindDependencyMacro)

@ONEQ_CONFIG_FIND_DEPENDENCIES@

include("${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@Targets.cmake")

if(MSVC AND TARGET @PROJECT_NAME@::@PROJECT_NAME_LOWER@)
    set_property(TARGET @PROJECT_NAME@::@PROJECT_NAME_LOWER@ PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

check_required_components(@PROJECT_NAME@)

set(@PROJECT_NAME@_VERSION @PROJECT_VERSION@)
set(@PROJECT_NAME@_INCLUDE_DIRS "@PACKAGE_CMAKE_INSTALL_INCLUDEDIR@")
set(@PROJECT_NAME@_LIBRARIES @PROJECT_NAME@::@PROJECT_NAME_LOWER@)
]=])

    configure_package_config_file(
        "${_oneq_config_template}"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
        PATH_VARS CMAKE_INSTALL_INCLUDEDIR
    )

    write_basic_package_version_file(
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    install(FILES
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
endfunction()
