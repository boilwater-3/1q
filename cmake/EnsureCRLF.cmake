# EnsureCRLF.cmake
# VS2015 (MSVC 190) has a known parser bug: LF-only source files containing
# multi-byte UTF-8 characters (e.g. Chinese comments) cause line-count
# misalignment, leading to spurious preprocessor and member-resolution errors.
# Additionally, UTF-8 multi-byte sequences inside string literals are
# misinterpreted under the system code page (GBK/936), breaking tokenisation.
#
# This module converts all .cpp/.h/.hpp files under src/, include/, and tests/
# from LF to CRLF and adds a UTF-8 BOM at configure time.  The BOM forces
# VS2015 to parse the file as UTF-8, preventing GBK misinterpretation of
# multi-byte characters in both comments and string literals.
#
# It is a no-op when the files already use CRLF with BOM or when the generator
# is not Visual Studio 14 2015.
#
# CMake file(READ/WRITE) uses text mode on Windows: READ strips \r, WRITE
# re-adds \r before \n.  A plain read-then-write round-trip therefore
# normalises any line ending to CRLF without manual string(REPLACE).
#
# The conversion is transient: .gitattributes (eol=lf) normalises them back
# on the next git checkout, so the repo is not permanently affected.

if(NOT CMAKE_GENERATOR MATCHES "^Visual Studio 14 2015")
  return()
endif()

message(STATUS "EnsureCRLF: scanning source files for LF-only line endings")

file(GLOB_RECURSE _ensure_crlf_files
  "${CMAKE_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_SOURCE_DIR}/src/*.h"
  "${CMAKE_SOURCE_DIR}/src/*.hpp"
  "${CMAKE_SOURCE_DIR}/include/*.h"
  "${CMAKE_SOURCE_DIR}/include/*.hpp"
  "${CMAKE_SOURCE_DIR}/tests/*.cpp"
  "${CMAKE_SOURCE_DIR}/tests/*.h"
  "${CMAKE_SOURCE_DIR}/tests/*.hpp"
)

# UTF-8 BOM (EF BB BF)
string(ASCII 239 187 191 _utf8_bom)

set(_ensure_crlf_count 0)
foreach(_file IN LISTS _ensure_crlf_files)
  # Read raw hex to detect LF-only files without text-mode \r stripping.
  file(READ "${_file}" _hex_content HEX)
  string(FIND "${_hex_content}" "0d0a" _crlf_pos)
  # Check if file already starts with UTF-8 BOM (EF BB BF).
  string(FIND "${_hex_content}" "efbbbf" _bom_pos)
  if(_crlf_pos LESS 0 OR NOT _bom_pos EQUAL 0)
    # File needs conversion: CRLF normalisation and/or BOM addition.
    file(READ "${_file}" _content)
    # Strip existing BOM to avoid producing a double BOM (EF BB BF EF BB BF).
    string(FIND "${_content}" "${_utf8_bom}" _existing_bom_pos)
    if(_existing_bom_pos EQUAL 0)
      string(LENGTH "${_utf8_bom}" _bom_len)
      string(SUBSTRING "${_content}" ${_bom_len} -1 _content)
    endif()
    file(WRITE "${_file}" "${_utf8_bom}${_content}")
    math(EXPR _ensure_crlf_count "${_ensure_crlf_count} + 1")
  endif()
endforeach()

if(_ensure_crlf_count GREATER 0)
  message(STATUS "EnsureCRLF: normalised ${_ensure_crlf_count} files to CRLF+UTF-8-BOM")
else()
  message(STATUS "EnsureCRLF: all files already use CRLF with BOM, nothing to do")
endif()

unset(_file)
unset(_content)
unset(_hex_content)
unset(_crlf_pos)
unset(_bom_pos)
unset(_utf8_bom)
unset(_ensure_crlf_files)
unset(_ensure_crlf_count)
