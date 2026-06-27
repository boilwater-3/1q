# check_docs_structure.cmake
#
# Guard repository documentation shape:
#   - docs/ has exactly common plus the five module directories
#   - each business module uses the five-file model
#   - common uses the approved common-document set
#   - top-level loose Markdown files and legacy archive/review/migration folders
#     do not reappear

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR is required")
endif()

set(ALLOWED_DOC_DIRS
    "airborne_radar"
    "common"
    "electro_optical_sensor"
    "electronic_surveillance_radar"
    "flight_dynamic"
    "sar")

set(BUSINESS_MODULE_DIRS
    "airborne_radar"
    "electro_optical_sensor"
    "electronic_surveillance_radar"
    "flight_dynamic"
    "sar")

set(MODULE_DOC_FILES
    "README.md"
    "design.md"
    "contract.md"
    "decisions.md"
    "history.md")

set(COMMON_DOC_FILES
    "README.md"
    "contract.md"
    "decisions.md"
    "history.md")

set(VIOLATIONS "")

file(GLOB _docs_children RELATIVE "${SOURCE_DIR}/docs" "${SOURCE_DIR}/docs/*")
foreach(child ${_docs_children})
  if(IS_DIRECTORY "${SOURCE_DIR}/docs/${child}")
    list(FIND ALLOWED_DOC_DIRS "${child}" _allowed_dir_idx)
    if(_allowed_dir_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/${child}: unexpected docs directory")
    endif()
  elseif(child MATCHES "\\.md$")
    list(APPEND VIOLATIONS "docs/${child}: top-level Markdown files are not allowed; place content under docs/common or a module directory")
  endif()
endforeach()

foreach(module ${BUSINESS_MODULE_DIRS})
  set(module_dir "${SOURCE_DIR}/docs/${module}")
  if(NOT IS_DIRECTORY "${module_dir}")
    list(APPEND VIOLATIONS "docs/${module}: missing module docs directory")
    continue()
  endif()

  foreach(filename ${MODULE_DOC_FILES})
    set(doc_file "${module_dir}/${filename}")
    if(NOT EXISTS "${doc_file}")
      list(APPEND VIOLATIONS "docs/${module}/${filename}: missing module doc")
      continue()
    endif()
    file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
    list(FIND _head "Status: active" _status_idx)
    if(_status_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/${module}/${filename}: must declare 'Status: active' near the top")
    endif()
  endforeach()

  file(GLOB_RECURSE _module_docs RELATIVE "${module_dir}" "${module_dir}/*.md")
  foreach(rel_doc ${_module_docs})
    list(FIND MODULE_DOC_FILES "${rel_doc}" _allowed_doc_idx)
    if(_allowed_doc_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/${module}/${rel_doc}: module docs must use README/design/contract/decisions/history only")
    endif()
  endforeach()
endforeach()

set(common_dir "${SOURCE_DIR}/docs/common")
if(NOT IS_DIRECTORY "${common_dir}")
  list(APPEND VIOLATIONS "docs/common: missing common docs directory")
else()
  foreach(filename ${COMMON_DOC_FILES})
    set(doc_file "${common_dir}/${filename}")
    if(NOT EXISTS "${doc_file}")
      list(APPEND VIOLATIONS "docs/common/${filename}: missing common doc")
      continue()
    endif()
    file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
    list(FIND _head "Status: active" _status_idx)
    if(_status_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/common/${filename}: must declare 'Status: active' near the top")
    endif()
  endforeach()

  file(GLOB_RECURSE _common_docs RELATIVE "${common_dir}" "${common_dir}/*.md")
  foreach(rel_doc ${_common_docs})
    list(FIND COMMON_DOC_FILES "${rel_doc}" _allowed_doc_idx)
    if(_allowed_doc_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/common/${rel_doc}: unexpected common doc")
    endif()
  endforeach()
endif()

if(VIOLATIONS)
  set(_err "Documentation structure violations:\n\n")
  foreach(v ${VIOLATIONS})
    string(APPEND _err "  ${v}\n")
  endforeach()
  message(FATAL_ERROR "${_err}")
endif()

message(STATUS "[docs-structure] common=4, modules=5, module_docs=25, violations=0")

