# check_sar_doc_governance.cmake
#
# Guard SAR documentation shape:
#   - docs/sar uses the design-doc set (design.md + boundaries.md + data-flow.md
#     + algorithms.md), all declaring Status: active
#   - legacy archive/contracts/audits/design/decisions/workflow directories do
#     not reappear under docs/sar
#
# Per docs/common/contract.md §文档结构, each business module keeps a design-doc
# set; the prior README/contract/decisions/history set has been collapsed into it.

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR is required")
endif()
# 规范化为绝对路径：file(RELATIVE_PATH) 与 GLOB ... RELATIVE 在 -P 脚本模式下，
# 当 base 为相对路径时分别报错或返回空列表。对绝对输入幂等，仅兜底手动调用。
get_filename_component(SOURCE_DIR "${SOURCE_DIR}" ABSOLUTE)

# design.md is mandatory; the other three are optional but must be active if present.
set(REQUIRED_ACTIVE_DOCS
    "docs/sar/design.md")

set(OPTIONAL_ACTIVE_DOCS
    "docs/sar/boundaries.md"
    "docs/sar/data-flow.md"
    "docs/sar/algorithms.md")

set(ALLOWED_SAR_DOCS
    "docs/sar/design.md"
    "docs/sar/boundaries.md"
    "docs/sar/data-flow.md"
    "docs/sar/algorithms.md")

set(FORBIDDEN_SAR_PATHS
    "docs/sar/archive"
    "docs/sar/audits"
    "docs/sar/contracts"
    "docs/sar/design"
    "docs/sar/decisions"
    "docs/sar/workflow")

set(VIOLATIONS "")

# Mandatory docs must exist and declare Status: active.
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

# Optional docs must declare Status: active if present; absence is allowed.
foreach(rel_path ${OPTIONAL_ACTIVE_DOCS})
  set(doc_file "${SOURCE_DIR}/${rel_path}")
  if(EXISTS "${doc_file}")
    file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
    list(FIND _head "Status: active" _status_idx)
    if(_status_idx EQUAL -1)
      list(APPEND VIOLATIONS "${rel_path}: SAR doc must declare 'Status: active' near the top")
    endif()
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
  list(FIND ALLOWED_SAR_DOCS "${rel_path}" _allowed_idx)
  if(_allowed_idx EQUAL -1)
    list(APPEND VIOLATIONS "${rel_path}: SAR docs must use the design-doc set (design.md/boundaries.md/data-flow.md/algorithms.md)")
  endif()
endforeach()

# No SAR doc may reference a removed legacy path.
foreach(rel_path ${ALLOWED_SAR_DOCS})
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
message(STATUS "[sar-doc-governance] required=${_active_count}, shape=design-doc-set, violations=0")

