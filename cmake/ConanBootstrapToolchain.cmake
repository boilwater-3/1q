include("${CMAKE_CURRENT_LIST_DIR}/ProjectLanguageDefaults.cmake")

# CMake 在编译器探测阶段会进入 try_compile 子工程，此时不应再次触发 Conan 安装。
get_property(_conan_in_try_compile GLOBAL PROPERTY IN_TRY_COMPILE)
if(_conan_in_try_compile)
  return()
endif()

# 统一 Conan 源目录和输出目录。
get_filename_component(_conan_source_dir "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(_conan_conanfile_path "${_conan_source_dir}/conanfile.py")

if(NOT EXISTS "${_conan_conanfile_path}")
    message(FATAL_ERROR "[Conan] conanfile.py not found: ${_conan_conanfile_path}")
endif()

file(SHA256 "${_conan_conanfile_path}" _conan_conanfile_hash)

# 构建类型推断规则：
#   - 单配置生成器：优先使用 CMAKE_BUILD_TYPE
#   - Conan 预设显式给出 CONAN_BUILD_TYPE 时，优先采用它
#   - 其他情况默认 Release
if(DEFINED CONAN_BUILD_TYPE AND NOT CONAN_BUILD_TYPE STREQUAL "")
    set(_conan_build_type "${CONAN_BUILD_TYPE}")
elseif(CMAKE_BUILD_TYPE)
    set(_conan_build_type "${CMAKE_BUILD_TYPE}")
else()
    set(_conan_build_type "Release")
endif()

# 统一 C++ 标准来源：优先使用外部传入，其次使用项目默认值
if(DEFINED CMAKE_CXX_STANDARD AND NOT CMAKE_CXX_STANDARD STREQUAL "")
    set(_conan_cppstd "${CMAKE_CXX_STANDARD}")
else()
    set(_conan_cppstd "${PROJECT_DEFAULT_CXX_STANDARD}")
endif()

if(_conan_cppstd LESS 11)
    message(FATAL_ERROR "[Conan] CMAKE_CXX_STANDARD must be at least 11")
endif()

# Conan 对 MSVC 190 不接受 compiler.cppstd=11，需要映射为 14
if(WIN32 AND CMAKE_GENERATOR MATCHES "^Visual Studio 14 2015$")
    if(_conan_cppstd STREQUAL "11")
        message(STATUS "[Conan] Mapping C++11 to compiler.cppstd=14 for MSVC 190 / VS2015")
        set(_conan_cppstd "14")
    endif()
endif()

# Conan 生成目录在单配置和 Visual Studio 多配置下不同。
set(_conan_output_dir "${CMAKE_BINARY_DIR}")
if(_conan_output_dir MATCHES "^(.*)/CMakeFiles/CMakeScratch/.*$")
    set(_conan_output_dir "${CMAKE_MATCH_1}")
endif()

set(_conan_conanfile_stamp "${_conan_output_dir}/build/conanfile.py.sha256")
set(_conan_install_fingerprint_stamp "${_conan_output_dir}/build/conan.install.fingerprint")

if(DEFINED ENABLE_TESTING)
    if(ENABLE_TESTING)
        set(_conan_enable_testing "True")
    else()
        set(_conan_enable_testing "False")
    endif()
else()
    set(_conan_enable_testing "False")
endif()

set(_conan_install_fingerprint
    "conanfile=${_conan_conanfile_hash};build_type=${_conan_build_type};cppstd=${_conan_cppstd};enable_testing=${_conan_enable_testing};generator=${CMAKE_GENERATOR};platform=${CMAKE_GENERATOR_PLATFORM};toolset=${CMAKE_GENERATOR_TOOLSET}")

set(_conan_real_toolchain
    "${_conan_output_dir}/build/${_conan_build_type}/generators/conan_toolchain.cmake")
if(WIN32 AND CMAKE_GENERATOR MATCHES "^Visual Studio ")
    set(_conan_real_toolchain
        "${_conan_output_dir}/build/generators/conan_toolchain.cmake")
endif()

set(_conan_needs_install FALSE)
if(NOT EXISTS "${_conan_real_toolchain}")
    set(_conan_needs_install TRUE)
    message(STATUS "[Conan] conan_toolchain.cmake not found, running conan install...")
else()
    unset(_conan_toolchain_std_line)
    file(STRINGS "${_conan_real_toolchain}" _conan_toolchain_std_line
        REGEX "^[ \t]*set\\(CMAKE_CXX_STANDARD[ \t]+\"?[0-9]+\"?\\)")
    if(_conan_toolchain_std_line)
        string(REGEX MATCH "[0-9]+" _conan_toolchain_cppstd "${_conan_toolchain_std_line}")
        if(NOT _conan_toolchain_cppstd STREQUAL _conan_cppstd)
            set(_conan_needs_install TRUE)
            message(STATUS "[Conan] C++ standard mismatch: current=${_conan_toolchain_cppstd}, expected=${_conan_cppstd}. Reinstalling...")
        endif()
    else()
        set(_conan_needs_install TRUE)
        message(STATUS "[Conan] toolchain missing CMAKE_CXX_STANDARD, reinstalling...")
    endif()
endif()

if(NOT EXISTS "${_conan_conanfile_stamp}")
    set(_conan_needs_install TRUE)
else()
    file(READ "${_conan_conanfile_stamp}" _conan_previous_conanfile_hash)
    string(STRIP "${_conan_previous_conanfile_hash}" _conan_previous_conanfile_hash)
    if(NOT _conan_previous_conanfile_hash STREQUAL _conan_conanfile_hash)
        set(_conan_needs_install TRUE)
        message(STATUS "[Conan] conanfile.py changed, reinstalling...")
    endif()
endif()

if(NOT EXISTS "${_conan_install_fingerprint_stamp}")
    set(_conan_needs_install TRUE)
else()
    file(READ "${_conan_install_fingerprint_stamp}" _conan_previous_install_fingerprint)
    string(STRIP "${_conan_previous_install_fingerprint}" _conan_previous_install_fingerprint)
    if(NOT _conan_previous_install_fingerprint STREQUAL _conan_install_fingerprint)
        set(_conan_needs_install TRUE)
        message(STATUS "[Conan] install inputs changed, reinstalling...")
    endif()
endif()

if(_conan_needs_install)
    find_program(_conan_conan_exe conan)
    if(NOT _conan_conan_exe)
        message(FATAL_ERROR
            "[Conan] 未找到 conan 可执行文件，请先安装 Conan 2.x（pip install conan）")
    endif()

set(_conan_install_command
    "${_conan_conan_exe}"
    install
    "${_conan_source_dir}"
    --output-folder
    "${_conan_output_dir}"
    --build
    missing
    -s
    "build_type=${_conan_build_type}"
    -s
    "compiler.cppstd=${_conan_cppstd}"
    -o
    "&:enable_testing=${_conan_enable_testing}")

    if(WIN32)
        set(_conan_arch "x86_64")
        if(CMAKE_GENERATOR_PLATFORM STREQUAL "Win32")
            set(_conan_arch "x86")
        elseif(CMAKE_GENERATOR_PLATFORM STREQUAL "ARM64")
            set(_conan_arch "armv8")
        endif()

        if(DEFINED CONAN_COMPILER_VERSION AND NOT CONAN_COMPILER_VERSION STREQUAL "")
            set(_conan_compiler_version "${CONAN_COMPILER_VERSION}")
        elseif(CMAKE_GENERATOR MATCHES "^Visual Studio 14 2015")
            set(_conan_compiler_version "190")
            set(_conan_vs_version "14")
        elseif(CMAKE_GENERATOR MATCHES "^Visual Studio 15 2017")
            set(_conan_compiler_version "191")
            set(_conan_vs_version "15")
        elseif(CMAKE_GENERATOR MATCHES "^Visual Studio 16 2019")
            set(_conan_compiler_version "192")
            set(_conan_vs_version "16")
        elseif(CMAKE_GENERATOR MATCHES "^Visual Studio 17 2022")
            set(_conan_compiler_version "194")
            set(_conan_vs_version "17")
        endif()

        if(DEFINED CONAN_COMPILER_RUNTIME AND NOT CONAN_COMPILER_RUNTIME STREQUAL "")
            set(_conan_runtime "${CONAN_COMPILER_RUNTIME}")
        else()
            set(_conan_runtime "dynamic")
        endif()

        if(_conan_build_type STREQUAL "Debug")
            set(_conan_runtime_type "Debug")
        else()
            set(_conan_runtime_type "Release")
        endif()

        if(NOT DEFINED _conan_compiler_version)
            message(FATAL_ERROR
                "[Conan] Unable to infer Conan MSVC version from generator: ${CMAKE_GENERATOR}")
        endif()

        list(APPEND _conan_install_command
            -s "os=Windows"
            -s "arch=${_conan_arch}"
            -s "compiler=msvc"
            -s "compiler.version=${_conan_compiler_version}"
            -s "compiler.runtime=${_conan_runtime}"
            -s "compiler.runtime_type=${_conan_runtime_type}"
            -c "tools.cmake.cmaketoolchain:generator=${CMAKE_GENERATOR}")

        if(DEFINED _conan_vs_version)
            list(APPEND _conan_install_command
                -c "tools.microsoft.msbuild:vs_version=${_conan_vs_version}")
        endif()

        if(CMAKE_GENERATOR_TOOLSET MATCHES "host=x64")
            list(APPEND _conan_install_command
                -c "tools.cmake.cmaketoolchain:toolset_arch=x64")
        endif()
    endif()

    execute_process(
        COMMAND ${_conan_install_command}
        WORKING_DIRECTORY "${_conan_source_dir}"
        RESULT_VARIABLE _conan_conan_result
        COMMAND_ECHO STDOUT
    )

    if(NOT _conan_conan_result EQUAL 0)
        message(FATAL_ERROR "[Conan] conan install failed (exit code: ${_conan_conan_result})")
    endif()

    file(WRITE "${_conan_conanfile_stamp}" "${_conan_conanfile_hash}\n")
    file(WRITE "${_conan_install_fingerprint_stamp}" "${_conan_install_fingerprint}\n")
endif()

if(NOT EXISTS "${_conan_real_toolchain}")
    message(FATAL_ERROR
        "[Conan] conan install finished but toolchain is still missing:\n  ${_conan_real_toolchain}")
endif()

include("${_conan_real_toolchain}")
