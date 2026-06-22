if(NOT DEFINED SOURCE_DIR OR NOT DEFINED CXX_COMPILER OR NOT DEFINED EIGEN_INCLUDE_DIR)
  message(FATAL_ERROR "SOURCE_DIR, CXX_COMPILER, and EIGEN_INCLUDE_DIR are required")
endif()

include("${SOURCE_DIR}/src/sar/SarSources.cmake")

set(OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/sar_cxx11_compat")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

foreach(SOURCE IN LISTS SAR_CXX11_COMPAT_SOURCES)
  get_filename_component(OBJECT_NAME "${SOURCE}" NAME_WE)
  if(CXX_COMPILER_ID STREQUAL "MSVC")
    set(COMPILE_COMMAND
        "${CXX_COMPILER}"
        /std:c++14
        /utf-8
        /EHsc
        "/I${SOURCE_DIR}/include"
        "/I${SOURCE_DIR}/src"
        "/I${EIGEN_INCLUDE_DIR}"
        /W4
        /WX
        /c "${SOURCE_DIR}/src/${SOURCE}"
        "/Fo${OUTPUT_DIR}/${OBJECT_NAME}.obj")
  else()
    set(COMPILE_COMMAND
        "${CXX_COMPILER}"
        -std=c++11
        "-I${SOURCE_DIR}/include"
        "-I${SOURCE_DIR}/src"
        -isystem
        "${EIGEN_INCLUDE_DIR}"
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -c "${SOURCE_DIR}/src/${SOURCE}"
        -o "${OUTPUT_DIR}/${OBJECT_NAME}.o")
  endif()
  execute_process(
      COMMAND ${COMPILE_COMMAND}
      RESULT_VARIABLE COMPILE_RESULT
      OUTPUT_VARIABLE COMPILE_STDOUT
      ERROR_VARIABLE COMPILE_STDERR)
  if(NOT COMPILE_RESULT EQUAL 0)
    message(FATAL_ERROR
            "SAR C++11 compatibility compile failed for ${SOURCE}\n"
            "${COMPILE_STDOUT}\n${COMPILE_STDERR}")
  endif()
endforeach()
