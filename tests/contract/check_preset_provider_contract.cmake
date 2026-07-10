# Keep public CMake presets aligned with the supported dependency provider and
# bootstrap entry point. A present preset is a supported user-facing contract.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(PRESETS_FILE "${SOURCE_DIR}/CMakePresets.json")
set(BOOTSTRAP_FILE "${SOURCE_DIR}/scripts/bootstrap_conan.sh")
file(READ "${PRESETS_FILE}" PRESETS_CONTENT)
file(READ "${BOOTSTRAP_FILE}" BOOTSTRAP_CONTENT)

if(PRESETS_CONTENT MATCHES "\"PACKAGE_MANAGER\"[ \t\r\n]*:[ \t\r\n]*\"none\"")
    message(FATAL_ERROR
        "CMakePresets.json exposes PACKAGE_MANAGER=none, but the project has no supported offline provider.")
endif()

foreach(required_preset IN ITEMS
        llvm-ninja-debug
        llvm-ninja-coverage
        llvm-ninja-release)
    if(NOT PRESETS_CONTENT MATCHES "\"name\"[ \t\r\n]*:[ \t\r\n]*\"${required_preset}\"")
        message(FATAL_ERROR "Required configure preset is missing: ${required_preset}")
    endif()
    if(NOT BOOTSTRAP_CONTENT MATCHES "${required_preset}\\)")
        message(FATAL_ERROR
            "bootstrap_conan.sh does not support required preset: ${required_preset}")
    endif()
endforeach()

execute_process(
    COMMAND "${CMAKE_COMMAND}" -S "${SOURCE_DIR}" --list-presets=all
    RESULT_VARIABLE list_presets_result
    OUTPUT_VARIABLE list_presets_output
    ERROR_VARIABLE list_presets_error)
if(NOT list_presets_result EQUAL 0)
    message(FATAL_ERROR
        "CMake could not parse CMakePresets.json:\n${list_presets_output}\n${list_presets_error}")
endif()

message(STATUS "[preset-provider] public presets and bootstrap entry point are consistent")
