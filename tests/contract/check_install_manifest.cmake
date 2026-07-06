# check_install_manifest.cmake
#
# 守护 install 清单的一致性。两层校验,均为 HARD 阻断:
#
# (1) install 清单 ↔ 磁盘头文件
#     public API boundary guard (check_public_api_boundary.cmake) 只校验
#     "whitelist == include/1q/ 磁盘头",但不校验 "install(FILES ...) 清单 == 磁盘头"。
#     install 清单是各 src/*/CMakeLists.txt 里逐个列举的,无目录通配,容易 drift:
#       - install 引用不存在的头(幽灵头,make install 失败)
#       - 磁盘存在但漏 install 的头(consumer find_package 后不可见)
#
# (2) install 清单 ↔ public API boundary whitelist
#     上述守护分别校验 install↔磁盘 与 whitelist↔磁盘,但二者之间无相互校验。
#     新增交叉检查:确保 install 清单与 boundary whitelist 描述同一组头文件,
#     杜绝 "装了却未声明为公开合同" 或 "声明了却未安装" 的漂移。
#
# 本守护扫描所有 src/*/CMakeLists.txt 中形如
#   ${CMAKE_SOURCE_DIR}/include/1q/<rest>
# 的 install 候选项,与 include/1q/ 实际头文件比对。不一致即 FATAL_ERROR。
# 随后从 check_public_api_boundary.cmake 提取 whitelist 路径做交叉比对。

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
        # 匹配形如 include/1q/foundation/pose_types.h 的路径。
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

# ── 交叉一致性:install 清单 ↔ public API boundary whitelist ──────────────
# 上述守护分别校验 "install↔磁盘" 与 "whitelist↔磁盘",但二者之间无相互
# 校验。新增一个顶层检查:确保 install 清单与 boundary whitelist 描述同一组
# 头文件,杜绝 "装了却未声明为公开合同" 或 "声明了却未安装" 的漂移。
#
# 从 check_public_api_boundary.cmake 提取它列举的所有相对路径(形如
# "模块/.../Foo.h"),加上 include/1q/ 前缀,转成与 INSTALLED_HEADERS 相同
# 的形式后比对。
set(BOUNDARY_SCRIPT "${SOURCE_DIR}/tests/contract/check_public_api_boundary.cmake")
file(STRINGS "${BOUNDARY_SCRIPT}" _boundary_lines)
set(BOUNDARY_HEADERS "")
foreach(line ${_boundary_lines})
    # 匹配形如 "airborne_radar/config/ArHardwareConfig.h" 或根级 "api.hpp"
    # 的带引号相对路径。路径段可含 0 个或多个 '/',扩展名为 .h/.hpp。
    # 小写/大写字母开头避免误匹配纯消息字符串。
    string(REGEX MATCHALL "\"[A-Za-z][A-Za-z0-9_/]+\\.(hpp|h)\"" _path_matches "${line}")
    foreach(m ${_path_matches})
        # 去掉首尾引号
        string(REGEX REPLACE "^\"" "" m "${m}")
        string(REGEX REPLACE "\"$" "" m "${m}")
        list(APPEND BOUNDARY_HEADERS "include/1q/${m}")
    endforeach()
endforeach()
list(REMOVE_DUPLICATES BOUNDARY_HEADERS)
list(SORT BOUNDARY_HEADERS)

# install 清单未包含但 boundary 声明的头(声明了却未安装)
set(WHITELIST_NOT_INSTALLED "")
foreach(h ${BOUNDARY_HEADERS})
    list(FIND INSTALLED_HEADERS "${h}" idx)
    if(idx EQUAL -1)
        list(APPEND WHITELIST_NOT_INSTALLED "${h}")
    endif()
endforeach()

# install 清单包含但 boundary 未声明的头(安装了却未列为公开合同)
set(INSTALLED_NOT_WHITELISTED "")
foreach(h ${INSTALLED_HEADERS})
    list(FIND BOUNDARY_HEADERS "${h}" idx)
    if(idx EQUAL -1)
        list(APPEND INSTALLED_NOT_WHITELISTED "${h}")
    endif()
endforeach()

set(CROSS_ERROR_MSG "")
if(WHITELIST_NOT_INSTALLED)
    string(APPEND CROSS_ERROR_MSG
        "\n[install-manifest] 以下头文件被 check_public_api_boundary.cmake 列为公开合同\n"
        "但未出现在任何 install 清单中(consumer 无法安装使用):\n")
    foreach(h ${WHITELIST_NOT_INSTALLED})
        string(APPEND CROSS_ERROR_MSG "  WHITELIST_NOT_INSTALLED  ${h}\n")
    endforeach()
endif()

if(INSTALLED_NOT_WHITELISTED)
    string(APPEND CROSS_ERROR_MSG
        "\n[install-manifest] 以下头文件出现在 install 清单中\n"
        "但未被 check_public_api_boundary.cmake 列为公开合同:\n")
    foreach(h ${INSTALLED_NOT_WHITELISTED})
        string(APPEND CROSS_ERROR_MSG "  INSTALLED_NOT_WHITELISTED  ${h}\n")
    endforeach()
endif()

if(CROSS_ERROR_MSG)
    message(FATAL_ERROR
        "${CROSS_ERROR_MSG}\n"
        "install 清单与 public API boundary whitelist 不一致。\n"
        "修复:新增公开头时须同时更新 src/*/CMakeLists.txt 的 install 清单\n"
        "与 tests/contract/check_public_api_boundary.cmake 的 whitelist。")
endif()

list(LENGTH DISK_HEADERS disk_count)
list(LENGTH INSTALLED_HEADERS installed_count)
list(LENGTH BOUNDARY_HEADERS whitelist_count)
message(STATUS
    "[install-manifest] 通过:磁盘 ${disk_count} 个头全部与 install 清单一致 "
    "(install 清单引用 ${installed_count} 个,boundary whitelist 声明 ${whitelist_count} 个,三者一致)。")
