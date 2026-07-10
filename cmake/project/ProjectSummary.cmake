# Project configuration summary. Kept separate from setup so the top-level
# lifecycle can emit its final state after all product directories are added.

macro(oneq_print_project_summary)
    message(STATUS "Configuration Summary")
    message(STATUS "  └─ Project: ${PROJECT_NAME} ${PROJECT_VERSION}")
    message(STATUS "  └─ C++ Standard: C++${CMAKE_CXX_STANDARD}")
    message(STATUS "  └─ Build shared libs: ${BUILD_SHARED_LIBS}")
    message(STATUS "  └─ Build examples: ${ENABLE_EXAMPLES}")
    message(STATUS "  └─ Build tests: ${ENABLE_TESTING}")
    message(STATUS "  └─ Coverage: ${ENABLE_COVERAGE}")
    message(STATUS "  └─ Install support: ${ENABLE_INSTALL}")
endmacro()
