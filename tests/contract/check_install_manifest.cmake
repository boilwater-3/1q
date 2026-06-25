# check_install_manifest.cmake
#
# 守护 install 清单与磁盘头文件的一致性。
#
# 背景:public API boundary guard (check_public_api_boundary.cmake) 只校验
# "whitelist == include/1q/ 磁盘头",但不校验 "install(FILES ...) 清单 == 磁盘头"。
# install 清单是各 src/*/CMakeLists.txt 里逐个列举的,无目录通配,容易 drift:
#   - install 引用不存在的头(幽灵头,make install 失败)
#   - 磁盘存在但漏 install 的头(consumer find_package 后不可见)
#
# 本守护扫描所有 src/*/CMakeLists.txt 中形如
#   ${CMAKE_SOURCE_DIR}/include/1q/<rest>
# 的 install 候选项,与 include/1q/ 实际头文件比对。不一致即 FATAL_ERROR。
#
# 这是 HARD 阻断:与 public_api_boundary_guard 同级,防止 install 真相源再次 drift。

cmake_minimum_required(VERSION 3.16)

# ---- 收集磁盘实际头文件 ----
# 注意:路径须带 "include/" 前缀,与下方 install 清单提取的路径形式一致
# (install 清单中写的是 include/1q/<rest>,而非 1q/<rest>)。
file(GLOB_RECURSE DISK_HEADERS
    LIST_DIRECTORIES FALSE
    "${SOURCE_DIR}/include/1q/*.h"
    "${SOURCE_DIR}/include/1q/*.hpp"
)
# GLOB_RECURSE 返回绝对路径;转成 "include/1q/<rest>" 形式。
set(DISK_HEADERS_REL "")
foreach(abs_path ${DISK_HEADERS})
    file(TO_CMAKE_PATH "${abs_path}" norm_path)
    string(FIND "${norm_path}" "/include/1q/" idx)
    if(idx EQUAL -1)
        continue()
    endif()
    math(EXPR start "${idx} + 1")
    string(SUBSTRING "${norm_path}" ${start} -1 rel_path)
    list(APPEND DISK_HEADERS_REL "${rel_path}")
endforeach()
set(DISK_HEADERS ${DISK_HEADERS_REL})
list(SORT DISK_HEADERS)

# ---- 收集 install 清单引用的头文件 ----
# 扫描两类来源:
#   1. 各模块 src/*/CMakeLists.txt 的 set(PUBLIC_HEADERS_*) 列表
#   2. cmake/*.cmake 中安装的根头(如 api.php,见 ProjectInstall.cmake)
file(GLOB MODULE_CMAKES "${SOURCE_DIR}/src/*/CMakeLists.txt")
file(GLOB CMAKE_MODULE_FILES "${SOURCE_DIR}/cmake/*.cmake")
set(MANIFEST_SOURCES ${MODULE_CMAKES} ${CMAKE_MODULE_FILES})

set(INSTALLED_HEADERS "")
foreach(cmake_file ${MANIFEST_SOURCES})
    file(STRINGS "${cmake_file}" lines)
    foreach(line ${lines})
        # 匹配形如 include/1q/foundation/json_reader.h 的路径。
        # 字符类 [A-Za-z0-9_/] 不含 '.',确保不跨越扩展名点号;
        # 这样 'airborne_radar.hpp' 只会被整体匹配,不会误匹配出 'airborne_radar.h'。
        # hpp 须在 h 之前,保证优先匹配长扩展名。
        string(REGEX MATCHALL "include/1q/[A-Za-z0-9_/]+\\.(hpp|h)" matches "${line}")
        foreach(m ${matches})
            list(APPEND INSTALLED_HEADERS "${m}")
        endforeach()
    endforeach()
endforeach()

list(REMOVE_DUPLICATES INSTALLED_HEADERS)
list(SORT INSTALLED_HEADERS)

# ---- 比对 ----
# 缺失安装:磁盘存在但 install 清单未引用。
set(MISSING_INSTALL "")
foreach(h ${DISK_HEADERS})
    list(FIND INSTALLED_HEADERS "${h}" idx)
    if(idx EQUAL -1)
        list(APPEND MISSING_INSTALL "${h}")
    endif()
endforeach()

# 幽灵安装:install 清单引用但磁盘不存在。
set(GHOST_INSTALL "")
foreach(h ${INSTALLED_HEADERS})
    list(FIND DISK_HEADERS "${h}" idx)
    if(idx EQUAL -1)
        list(APPEND GHOST_INSTALL "${h}")
    endif()
endforeach()

set(ERROR_MSG "")
if(MISSING_INSTALL)
    string(APPEND ERROR_MSG
        "\n[install-manifest] 以下头文件存在于 include/1q/ 但未出现在任何 install 清单中\n"
        "(consumer 经 find_package 后无法包含,需在对应 src/*/CMakeLists.txt 的 set(PUBLIC_HEADERS_*) 中补齐):\n")
    foreach(h ${MISSING_INSTALL})
        string(APPEND ERROR_MSG "  MISSING_INSTALL  ${h}\n")
    endforeach()
endif()

if(GHOST_INSTALL)
    string(APPEND ERROR_MSG
        "\n[install-manifest] 以下头文件被 install 清单引用但磁盘不存在(幽灵头,make install 会失败):\n")
    foreach(h ${GHOST_INSTALL})
        string(APPEND ERROR_MSG "  GHOST_INSTALL    ${h}\n")
    endforeach()
endif()

if(ERROR_MSG)
    message(FATAL_ERROR
        "${ERROR_MSG}\n"
        "install 清单与 include/1q/ 磁盘头文件不一致。\n"
        "修复:确保每个 src/*/CMakeLists.txt 的 install(FILES \${PUBLIC_HEADERS_*}) 覆盖对应模块的全部 public 头。\n"
        "本守护与 check_public_api_boundary.cmake 互补:前者管 whitelist↔磁盘,本守护管 install↔磁盘。")
endif()

list(LENGTH DISK_HEADERS disk_count)
list(LENGTH INSTALLED_HEADERS installed_count)
message(STATUS
    "[install-manifest] 通过:磁盘 ${disk_count} 个头全部与 install 清单一致 "
    "(install 清单引用 ${installed_count} 个)。")
