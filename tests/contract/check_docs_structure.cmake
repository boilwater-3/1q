# check_docs_structure.cmake
#
# Guard repository documentation shape:
#   - docs/ has common, review, and the five module directories
#   - each business module uses the single-file design.md model
#   - common uses the approved common-document set (contract.md + open_questions.md)
#   - review uses draft Markdown files only, with no nested directory tree
#   - top-level loose Markdown files and legacy archive/migration folders do not reappear
#
# Per docs/common/contract.md §文档结构, each business module keeps only
# design.md as its design authority; the prior README/contract/decisions/history
# set has been collapsed into design.md, and common keeps contract.md (public
# contract) plus open_questions.md (non-normative cross-module open questions).
# docs/review is the only approved draft/review holding area.

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
    "review"
    "sar")

set(BUSINESS_MODULE_DIRS
    "airborne_radar"
    "electro_optical_sensor"
    "electronic_surveillance_radar"
    "flight_dynamic"
    "sar")

set(MODULE_DOC_FILES
    "design.md")

set(COMMON_DOC_FILES
    "contract.md"
    "open_questions.md")

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
      list(APPEND VIOLATIONS "docs/${module}/${rel_doc}: module docs must use design.md only")
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

set(review_dir "${SOURCE_DIR}/docs/review")
if(IS_DIRECTORY "${review_dir}")
  file(GLOB_RECURSE _review_docs RELATIVE "${review_dir}" "${review_dir}/*.md")
  foreach(rel_doc ${_review_docs})
    if(rel_doc MATCHES "/")
      list(APPEND VIOLATIONS "docs/review/${rel_doc}: review docs must stay flat")
    endif()
    set(doc_file "${review_dir}/${rel_doc}")
    file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
    list(FIND _head "Status: draft" _draft_idx)
    if(_draft_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/review/${rel_doc}: review docs must declare 'Status: draft' near the top")
    endif()
  endforeach()

  file(GLOB _review_children RELATIVE "${review_dir}" "${review_dir}/*")
  foreach(child ${_review_children})
    if(IS_DIRECTORY "${review_dir}/${child}")
      list(APPEND VIOLATIONS "docs/review/${child}: review subdirectories are not allowed")
    elseif(NOT child MATCHES "\\.md$")
      list(APPEND VIOLATIONS "docs/review/${child}: review entries must be Markdown files")
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

list(LENGTH COMMON_DOC_FILES _common_count)
list(LENGTH MODULE_DOC_FILES _module_doc_count)
list(LENGTH BUSINESS_MODULE_DIRS _module_count)
if(IS_DIRECTORY "${SOURCE_DIR}/docs/review")
  file(GLOB _review_docs_count "${SOURCE_DIR}/docs/review/*.md")
  list(LENGTH _review_docs_count _review_count)
else()
  set(_review_count 0)
endif()
math(EXPR _module_doc_total "${_module_count} * ${_module_doc_count}")
message(STATUS "[docs-structure] common=${_common_count}, modules=${_module_count}, module_docs=${_module_doc_total}, review=${_review_count}, violations=0")
