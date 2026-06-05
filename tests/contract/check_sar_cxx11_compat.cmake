if(NOT DEFINED SOURCE_DIR OR NOT DEFINED CXX_COMPILER OR NOT DEFINED EIGEN_INCLUDE_DIR)
  message(FATAL_ERROR "SOURCE_DIR, CXX_COMPILER, and EIGEN_INCLUDE_DIR are required")
endif()

set(SAR_ENGINE_SOURCES
    sar/echo/SarEcho.cpp
    sar/geometry/SarGeometry.cpp
    sar/imaging/SarGbp.cpp
    sar/imaging/SarImageQuality.cpp
    sar/imaging/SarRda.cpp
    sar/runtime/PulseRingBuffer.cpp
    sar/signal/SarFft.cpp
    sar/signal/SarWaveform.cpp
)

set(OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/sar_cxx11_compat")
file(MAKE_DIRECTORY "${OUTPUT_DIR}")

foreach(SOURCE IN LISTS SAR_ENGINE_SOURCES)
  get_filename_component(OBJECT_NAME "${SOURCE}" NAME_WE)
  execute_process(
      COMMAND "${CXX_COMPILER}"
              -std=c++11
              "-I${SOURCE_DIR}/src"
              -isystem
              "${EIGEN_INCLUDE_DIR}"
              -Wall
              -Wextra
              -Wpedantic
              -Werror
              -c "${SOURCE_DIR}/src/${SOURCE}"
              -o "${OUTPUT_DIR}/${OBJECT_NAME}.o"
      RESULT_VARIABLE COMPILE_RESULT
      OUTPUT_VARIABLE COMPILE_STDOUT
      ERROR_VARIABLE COMPILE_STDERR)
  if(NOT COMPILE_RESULT EQUAL 0)
    message(FATAL_ERROR
            "SAR C++11 compatibility compile failed for ${SOURCE}\n"
            "${COMPILE_STDOUT}\n${COMPILE_STDERR}")
  endif()
endforeach()
