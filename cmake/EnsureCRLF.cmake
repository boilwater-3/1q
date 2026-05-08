# EnsureCRLF.cmake
# VS2015 (MSVC 190) has a known parser bug: LF-only source files containing
# multi-byte UTF-8 characters (e.g. Chinese comments) cause line-count
# misalignment, leading to spurious preprocessor and member-resolution errors.
#
# This module converts all .cpp/.h/.hpp files under src/ and include/ from
# LF to CRLF at configure time.  It is a no-op when the files already use CRLF
# or when the generator is not Visual Studio 14 2015.
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
)

set(_ensure_crlf_count 0)
foreach(_file IN LISTS _ensure_crlf_files)
  # Read raw hex to detect LF-only files without text-mode \r stripping.
  file(READ "${_file}" _hex_content HEX)
  string(FIND "${_hex_content}" "0d0a" _crlf_pos)
  if(_crlf_pos LESS 0)
    # File is LF-only.  Read in text mode (strips \r), write in text mode
    # (re-adds \r before \n) — net effect: LF → CRLF.
    file(READ "${_file}" _content)
    file(WRITE "${_file}" "${_content}")
    math(EXPR _ensure_crlf_count "${_ensure_crlf_count} + 1")
  endif()
endforeach()

if(_ensure_crlf_count GREATER 0)
  message(STATUS "EnsureCRLF: converted ${_ensure_crlf_count} files to CRLF")
else()
  message(STATUS "EnsureCRLF: all files already use CRLF, nothing to do")
endif()

unset(_file)
unset(_content)
unset(_hex_content)
unset(_crlf_pos)
unset(_ensure_crlf_files)
unset(_ensure_crlf_count)
