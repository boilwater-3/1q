if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR is required")
endif()

include("${SOURCE_DIR}/src/sar/SarSources.cmake")

set(ACTIVE_SAR_SOURCES
    ${SAR_ENGINE_SOURCES}
    ${SAR_CORE_SOURCES}
)

set(FROZEN_SAR_SOURCE_PATTERNS
    "sar/imaging/SarCsa"
)

foreach(ACTIVE_SOURCE IN LISTS ACTIVE_SAR_SOURCES)
  foreach(FROZEN_PATTERN IN LISTS FROZEN_SAR_SOURCE_PATTERNS)
    if(ACTIVE_SOURCE MATCHES "^${FROZEN_PATTERN}")
      message(FATAL_ERROR
              "Frozen SAR source entered active build manifest: ${ACTIVE_SOURCE}")
    endif()
  endforeach()
endforeach()

