# check_sar_doc_governance.cmake
#
# Guard SAR documentation shape:
#   - docs/sar uses the five-file model only
#   - each required file declares Status: active near the top
#   - legacy archive/contracts/audits/design/decisions/workflow directories do
#     not reappear under docs/sar

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(REQUIRED_ACTIVE_DOCS
    "docs/sar/README.md"
    "docs/sar/design.md"
    "docs/sar/contract.md"
    "docs/sar/decisions.md"
    "docs/sar/history.md")

set(FORBIDDEN_SAR_PATHS
    "docs/sar/archive"
    "docs/sar/audits"
    "docs/sar/contracts"
    "docs/sar/design"
    "docs/sar/decisions"
    "docs/sar/workflow")

set(VIOLATIONS "")

foreach(rel_path ${REQUIRED_ACTIVE_DOCS})
  set(doc_file "${SOURCE_DIR}/${rel_path}")
  if(NOT EXISTS "${doc_file}")
    list(APPEND VIOLATIONS "${rel_path}: missing required SAR doc")
    continue()
  endif()

  file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
  list(FIND _head "Status: active" _status_idx)
  if(_status_idx EQUAL -1)
    list(APPEND VIOLATIONS "${rel_path}: required SAR doc must declare 'Status: active' near the top")
  endif()
endforeach()

foreach(rel_path ${FORBIDDEN_SAR_PATHS})
  if(EXISTS "${SOURCE_DIR}/${rel_path}")
    list(APPEND VIOLATIONS "${rel_path}: legacy SAR documentation directory must not exist")
  endif()
endforeach()

file(GLOB_RECURSE SAR_DOCS "${SOURCE_DIR}/docs/sar/*.md")
foreach(doc_file ${SAR_DOCS})
  file(RELATIVE_PATH rel_path "${SOURCE_DIR}" "${doc_file}")
  list(FIND REQUIRED_ACTIVE_DOCS "${rel_path}" _allowed_idx)
  if(_allowed_idx EQUAL -1)
    list(APPEND VIOLATIONS "${rel_path}: SAR docs must use the five-file model")
  endif()
endforeach()

foreach(rel_path ${REQUIRED_ACTIVE_DOCS})
  set(doc_file "${SOURCE_DIR}/${rel_path}")
  if(EXISTS "${doc_file}")
    file(STRINGS "${doc_file}" _lines)
    foreach(line ${_lines})
      foreach(forbidden ${FORBIDDEN_SAR_PATHS})
        string(FIND "${line}" "${forbidden}" _idx)
        if(NOT _idx EQUAL -1)
          list(APPEND VIOLATIONS "${rel_path}: references removed legacy path '${forbidden}'")
        endif()
      endforeach()
    endforeach()
  endif()
endforeach()

if(VIOLATIONS)
  set(_err "SAR documentation governance violations:\n\n")
  foreach(v ${VIOLATIONS})
    string(APPEND _err "  ${v}\n")
  endforeach()
  message(FATAL_ERROR "${_err}")
endif()

list(LENGTH REQUIRED_ACTIVE_DOCS _active_count)
message(STATUS "[sar-doc-governance] active=${_active_count}, shape=five-file, violations=0")

