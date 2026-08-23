/**
 * @file fs_compat.h
 * @brief std::filesystem 跨编译器别名（app_fs），示例装配层共用。
 *
 * std::filesystem 为 C++17；VS2015（msvc 190）下同头文件提供 TR2 的
 * std::experimental::filesystem，别名统一调用面。
 */

#ifndef EXAMPLES_APP_FS_COMPAT_H_
#define EXAMPLES_APP_FS_COMPAT_H_

#if defined(_MSC_VER) && _MSC_VER < 1910
#include <filesystem>
namespace app_fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace app_fs = std::filesystem;
#endif

#endif  // EXAMPLES_APP_FS_COMPAT_H_
