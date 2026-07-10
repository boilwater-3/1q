# Project option entry point. Keep all user-configurable build and module
# switches here so `cmake/project` has one physical option boundary.

option(BUILD_SHARED_LIBS "Build shared libraries instead of static" ON)
option(ENABLE_TESTING "Enable testing support" OFF)
option(ENABLE_EXAMPLES "Build example programs" OFF)
option(ENABLE_INSTALL "Enable installation rules" OFF)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
    option(ENABLE_UNITY_BUILD "Enable Unity Build for faster compilation" OFF)
else()
    set(ENABLE_UNITY_BUILD OFF CACHE INTERNAL "Unity Build requires CMake 3.16+")
endif()

if(UNIX)
    option(USE_CCACHE "Use ccache to accelerate rebuilds" ON)
else()
    set(USE_CCACHE OFF CACHE INTERNAL "ccache is primarily for Linux/macOS")
endif()

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
    option(ENABLE_PCH "Enable precompiled headers" OFF)
    mark_as_advanced(ENABLE_PCH)
else()
    set(ENABLE_PCH OFF CACHE INTERNAL "Precompiled headers require CMake 3.16+")
endif()

option(ENABLE_WARNINGS "Enable additional compiler warnings" ON)
option(ENABLE_COVERAGE "Enable LLVM source-based coverage instrumentation (Clang only)" OFF)
mark_as_advanced(ENABLE_COVERAGE)
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
mark_as_advanced(ENABLE_CLANG_TIDY)

set(CLANG_TIDY_CHECKS
    "-*,readability-braces-around-statements,readability-else-after-return,readability-isolate-declaration,readability-qualified-auto,readability-redundant-access-specifiers,readability-redundant-control-flow,readability-redundant-preprocessor,readability-static-accessed-through-instance,modernize-redundant-void-arg,modernize-use-bool-literals,modernize-use-default-member-init,modernize-use-equals-default,modernize-use-equals-delete,modernize-use-nullptr,modernize-use-override,modernize-use-using,performance-for-range-copy,performance-implicit-conversion-in-loop,performance-move-constructor-init,performance-unnecessary-copy-initialization,performance-unnecessary-value-param,bugprone-macro-parentheses,bugprone-string-constructor,bugprone-string-integer-assignment"
    CACHE STRING "clang-tidy checks whitelist")
mark_as_advanced(CLANG_TIDY_CHECKS)

set(STACK_SIZE_OPTION "RECOMMENDED" CACHE STRING "Stack size configuration")
set_property(CACHE STACK_SIZE_OPTION PROPERTY STRINGS
    "DEFAULT" "RECOMMENDED" "LARGE_PROJECT" "EXTREME_RECURSION")
mark_as_advanced(STACK_SIZE_OPTION)

set(PACKAGE_MANAGER "conan" CACHE STRING
    "1q dependency provider (Conan is the only supported value)")
set_property(CACHE PACKAGE_MANAGER PROPERTY STRINGS "conan")
if(NOT PACKAGE_MANAGER STREQUAL "conan")
    message(FATAL_ERROR
        "Unsupported PACKAGE_MANAGER: '${PACKAGE_MANAGER}'. "
        "1q currently supports only 'conan'; run scripts/bootstrap_conan.sh <preset> first.")
endif()
message(STATUS "Package Manager: Conan")

option(ONEQ_ENABLE_FLIGHT_DYNAMIC "Build the flight_dynamic (maneuver) module" OFF)
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    message(STATUS "flight_dynamic module: ENABLED")
else()
    message(STATUS "flight_dynamic module: disabled (set -DONEQ_ENABLE_FLIGHT_DYNAMIC=ON to enable)")
endif()
