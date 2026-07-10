# Third-party discovery entry point. It intentionally creates/imports provider
# targets only; business modules consume those targets in their own CMakeLists.

set(ONEQ_PROJECT_DEPENDENCIES_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(oneq_find_project_dependencies)
    include("${ONEQ_PROJECT_DEPENDENCIES_DIR}/dependencies/JsbsimProvider.cmake")
    include("${ONEQ_PROJECT_DEPENDENCIES_DIR}/dependencies/ConanPackages.cmake")

    foreach(_oneq_dependency_var IN ITEMS
            ONEQ_HAVE_ZLIB
            ONEQ_ENABLE_HDF5_OUTPUT
            ONEQ_LINK_DEPENDENCIES
            ONEQ_JSBSIM_BINARY_SOURCE
            ONEQ_JSBSIM_DATA_ROOT_DIR
            ONEQ_JSBSIM_DATA_SOURCE
            PROJECT_ENABLE_SPDLOG
            PROJECT_SPDLOG_TARGET)
        if(DEFINED ${_oneq_dependency_var})
            set(${_oneq_dependency_var} "${${_oneq_dependency_var}}" PARENT_SCOPE)
        endif()
    endforeach()
endfunction()
