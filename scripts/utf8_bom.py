#!/usr/bin/env python3
"""管理 git 追踪 C/C++ 源文件的 UTF-8 BOM。

背景：客户 VS2015 集成端按系统代码页（GBK）误读无 BOM 的 UTF-8 源文件，
导致中文注释错位截断；MSVC cl.exe 对带 BOM 的文件无条件按 UTF-8 解码，
不依赖编译选项、pragma 与 Update 等级，故仓库统一为 C/C++ 文件携带 BOM。
仅 C/C++ 扩展名加 BOM：shell 脚本 shebang 会因 BOM 失效，其余文本无必要。

子命令：
  convert  为 git 追踪的 C/C++ 文件前置 EF BB BF（幂等；仅追加这 3 字节，
           不触碰行尾与其余内容；内容须通过严格 UTF-8 校验，非法则跳过并报错）
  check    校验全部目标文件带 BOM 且为合法 UTF-8，违例列文件并退出码 1（CI 守卫）
  strip    剥离 BOM。默认原地剥离；--output-dir DIR 时改为镜像整个追踪文件树到
           DIR（仅 C/C++ 文件剥 BOM，其余原样复制）——Linux/GCC 移植遇到
           stray '\\357' 报错时的逃生通道，不动仓库工作区
"""

import argparse
import subprocess
import sys
from pathlib import Path

# 与 .editorconfig 的 utf-8-bom 段、CI check 步骤保持同一范围
SOURCE_EXTENSIONS = (
    ".h", ".hh", ".hpp", ".hxx", ".ipp", ".inl",
    ".c", ".cc", ".cpp", ".cxx",
)

UTF8_BOM = b"\xef\xbb\xbf"

# 仓库根目录（本脚本位于 scripts/ 下）
REPO_ROOT = Path(__file__).resolve().parent.parent


def is_source_file(path: str) -> bool:
    return path.endswith(SOURCE_EXTENSIONS)


def list_tracked_files() -> list:
    """返回 git 追踪文件列表（相对路径，/ 分隔）。"""
    output = subprocess.check_output(
        ["git", "-C", str(REPO_ROOT), "ls-files", "-z"], text=False
    )
    return [name for name in output.decode("utf-8").split("\0") if name]


def read_bytes(rel_path: str) -> bytes:
    return (REPO_ROOT / rel_path).read_bytes()


def write_bytes(rel_path: str, data: bytes, shadow_root=None) -> None:
    if shadow_root is None:
        target = REPO_ROOT / rel_path
    else:
        target = shadow_root / rel_path
        target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)


def cmd_convert(args) -> int:
    converted, skipped, invalid = [], [], []
    for name in list_tracked_files():
        if not is_source_file(name):
            continue
        data = read_bytes(name)
        if data.startswith(UTF8_BOM):
            skipped.append(name)
            continue
        try:
            data.decode("utf-8")
        except UnicodeDecodeError as exc:
            invalid.append((name, str(exc)))
            continue
        write_bytes(name, UTF8_BOM + data)
        converted.append(name)

    print(f"converted: {len(converted)}")
    print(f"skipped (BOM 已存在): {len(skipped)}")
    print(f"invalid (非合法 UTF-8，未写入): {len(invalid)}")
    for name, err in invalid:
        print(f"  error: {name}: {err}", file=sys.stderr)
    return 1 if invalid else 0


def cmd_check(args) -> int:
    violations = []
    for name in list_tracked_files():
        if not is_source_file(name):
            continue
        data = read_bytes(name)
        if not data.startswith(UTF8_BOM):
            violations.append((name, "缺少 UTF-8 BOM"))
            continue
        try:
            data[len(UTF8_BOM):].decode("utf-8")
        except UnicodeDecodeError as exc:
            violations.append((name, f"BOM 后内容非合法 UTF-8: {exc}"))

    if violations:
        for name, reason in violations:
            print(f"error: {name}: {reason}", file=sys.stderr)
        print(f"check 失败: {len(violations)} 个文件违例", file=sys.stderr)
        return 1
    print("check 通过: 全部 C/C++ 源文件均带 UTF-8 BOM 且为合法 UTF-8")
    return 0


def cmd_strip(args) -> int:
    stripped, untouched = 0, 0
    if args.output_dir:
        shadow_root = Path(args.output_dir).resolve()
        if shadow_root.exists() and any(shadow_root.iterdir()):
            print(f"error: 输出目录非空: {shadow_root}", file=sys.stderr)
            return 1
    else:
        shadow_root = None

    for name in list_tracked_files():
        data = read_bytes(name)
        if is_source_file(name) and data.startswith(UTF8_BOM):
            write_bytes(name, data[len(UTF8_BOM):], shadow_root=shadow_root)
            stripped += 1
        elif shadow_root is not None:
            # 影子模式需要完整可构建的树：非目标文件原样复制
            write_bytes(name, data, shadow_root=shadow_root)
            untouched += 1

    if shadow_root is not None:
        print(f"strip 完成: 剥离 {stripped} 个, 原样复制 {untouched} 个 -> {shadow_root}")
    else:
        print(f"strip 完成: 原地剥离 {stripped} 个文件的 BOM")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("convert", help="为 C/C++ 源文件前置 UTF-8 BOM（幂等）")
    sub.add_parser("check", help="校验全部 C/C++ 源文件带 BOM（CI 守卫）")

    p_strip = sub.add_parser("strip", help="剥离 BOM（原地或镜像到输出目录）")
    p_strip.add_argument(
        "--output-dir",
        default=None,
        help="镜像输出目录（须为空目录/不存在）：复制全部追踪文件，仅 C/C++ 剥 BOM",
    )

    args = parser.parse_args(argv)
    handlers = {"convert": cmd_convert, "check": cmd_check, "strip": cmd_strip}
    return handlers[args.command](args)


if __name__ == "__main__":
    # Windows 控制台默认代码页可能不是 UTF-8，避免打印中文文件名时抛异常
    try:
        sys.stdout.reconfigure(errors="replace")
        sys.stderr.reconfigure(errors="replace")
    except AttributeError:
        pass
    sys.exit(main())
