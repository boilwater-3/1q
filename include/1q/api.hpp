#pragma once

// Export macro placeholder; adjust when building shared libs if needed.
#ifndef TEMPLATE_API
#ifdef _WIN32
#if defined(TEMPLATE_LIBRARY_BUILD)
#define TEMPLATE_API __declspec(dllexport)
#elif defined(TEMPLATE_STATIC)
#define TEMPLATE_API
#elif defined(TEMPLATE_LIBRARY_SHARED)
#define TEMPLATE_API __declspec(dllimport)
#else
#define TEMPLATE_API __declspec(dllimport)
#endif
#else
#ifdef TEMPLATE_LIBRARY_BUILD
#define TEMPLATE_API __attribute__((visibility("default")))
#else
#define TEMPLATE_API
#endif
#endif
#endif
