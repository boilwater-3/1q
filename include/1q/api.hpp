#ifndef ONEQ_API_HPP_
#define ONEQ_API_HPP_

#if defined(__has_include)
#if __has_include("1q/oneq_export.h")
#include "1q/oneq_export.h"
#endif
#endif

/*
 * Fallback for non-CMake or pre-generated-header environments.
 * The normal build path uses GenerateExportHeader-generated ONEQ_EXPORT.
 */
#ifndef ONEQ_EXPORT
#ifdef _WIN32
#if defined(ONEQ_STATIC_DEFINE)
#define ONEQ_EXPORT
#else
#if defined(oneq_lib_EXPORTS)
#define ONEQ_EXPORT __declspec(dllexport)
#else
#define ONEQ_EXPORT __declspec(dllimport)
#endif
#endif
#else
#define ONEQ_EXPORT __attribute__((visibility("default")))
#endif
#endif

#ifndef ONEQ_API
#define ONEQ_API ONEQ_EXPORT
#endif

#endif // ONEQ_API_HPP_
