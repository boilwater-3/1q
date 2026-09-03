#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lint_algorithms_md.py - 1Q 算法文档规范检查器

用于自动化检查 docs/*/algorithms.md 是否遵循 1Q 算法展示标准规范。
用法:
    python lint_algorithms_md.py [path/to/algorithms.md ...]
    python lint_algorithms_md.py --all
    python lint_algorithms_md.py --check-paths
"""

import sys
import os
import re
import argparse
from pathlib import Path
from typing import List, Dict, Any, Tuple

# 确保在 Windows 控制台或管道中正确输出 UTF-8
if sys.stdout.encoding and sys.stdout.encoding.lower() != 'utf-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

VALID_STATUSES = {
    "session-wired",
    "characterized",
    "experimental",
    "internal/受控"
}

HISTORICAL_STATUS_MAP = {
    "生产可用": "session-wired",
    "实验接线": "experimental"
}

REQUIRED_FRONTMATTER_FIELDS = ["Status", "Last-reviewed", "Authority", "Answers"]

class LintResult:
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.info: List[str] = []
        self.algorithms_found: List[str] = []

    def add_error(self, msg: str):
        self.errors.append(msg)

    def add_warning(self, msg: str):
        self.warnings.append(msg)

    def add_info(self, msg: str):
        self.info.append(msg)

    @property
    def is_valid(self) -> bool:
        return len(self.errors) == 0

def parse_frontmatter(content: str) -> Tuple[Dict[str, str], str]:
    if not content.startswith("---"):
        return {}, content
    parts = content.split("---", 2)
    if len(parts) < 3:
        return {}, content
    yaml_text = parts[1]
    body = parts[2]
    fields = {}
    for line in yaml_text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" in line:
            key, val = line.split(":", 1)
            fields[key.strip()] = val.strip()
    return fields, body

def check_algorithms_table(body: str, result: LintResult):
    # 匹配算法登记表
    table_pattern = re.compile(r'##\s*(?:算法登记表|算法清单|算法登记)[\s\S]*?\n(\|.*?\|\n\|[-:\s|]+\|\n(?:\|.*?\|\n?)+)', re.MULTILINE)
    match = table_pattern.search(body)
    if not match:
        result.add_error("缺少 [## 算法登记表] 或表格格式不合规")
        return

    table_content = match.group(1).strip().splitlines()
    if len(table_content) < 3:
        result.add_error("算法登记表为空或格式损坏")
        return

    headers = [col.strip() for col in table_content[0].strip('|').split('|')]
    if len(headers) < 4:
        result.add_warning(f"算法登记表列数推荐为 4 列（当前为 {len(headers)} 列: {headers}）")

    rows = table_content[2:]
    for idx, row in enumerate(rows, start=1):
        cols = [c.strip() for c in row.strip('|').split('|')]
        if len(cols) >= 3:
            algo_name = cols[0]
            status = cols[2] if len(cols) == 4 else cols[-2]
            result.algorithms_found.append(algo_name)
            
            # 状态检查
            if status in HISTORICAL_STATUS_MAP:
                result.add_warning(f"行 {idx} [{algo_name}] 使用了旧状态词 '{status}'，建议迁移为标准枚举 '{HISTORICAL_STATUS_MAP[status]}'")
            elif status not in VALID_STATUSES and not any(s in status for s in VALID_STATUSES):
                result.add_warning(f"行 {idx} [{algo_name}] 状态 '{status}' 不属于推荐标准状态: {VALID_STATUSES}")

def check_algorithm_sections(body: str, result: LintResult):
    # 查找算法二级标题
    sections = re.findall(r'\n##\s+(?:\d+\.?\s*)?([^\n#]+)', body)
    reserved_headers = {"算法登记表", "算法清单", "算法登记", "核心算法详述", "辅助/映射类算法", "非目标", "非目标（刻意不实现的算法）", "公式与口径", "输出语义", "反直觉点"}
    
    algo_sections = [s.strip() for s in sections if s.strip() not in reserved_headers]
    
    # 检查反直觉点
    has_counter_intuitive = bool(re.search(r'反直觉|Counter-Intuitive', body, re.IGNORECASE))
    if not has_counter_intuitive:
        result.add_warning("文档未发现 [反直觉点] 章节/条款，反直觉点是 1Q 算法文档的核心资产")

    # 检查实现边界
    has_boundaries = bool(re.search(r'实现边界|边界在哪|边界与反直觉', body))
    if not has_boundaries:
        result.add_warning("文档未发现 [实现边界] 相关论述")

    # 检查证据标记
    evidence_matches = re.findall(r'\[evidence:\s*([^\]]+)\]', body)
    if not evidence_matches:
        raw_test_matches = re.findall(r'tests/(?:unit|contract|integration|consumer)/[^\s;`\)]+', body)
        if not raw_test_matches:
            result.add_error("未找到任何测试证据链锚点（需包含 [evidence: tests/...]）")
        else:
            result.add_info(f"找到 {len(raw_test_matches)} 处裸测试路径，建议包装为标准 [evidence: tests/...] 标签")
    else:
        result.add_info(f"找到 {len(evidence_matches)} 处规范证据标签")

def check_evidence_paths(body: str, repo_root: Path, result: LintResult):
    # 提取所有测试/契约文件路径（支持 .cpp, .h, .cmake, .md）
    test_paths = []
    for match in re.finditer(r'tests/[\w\-_/]+(?:\.cpp|\.h|\.cmake|\.md)?', body):
        test_paths.append(match.group(0))

    missing_paths = []
    for tp in set(test_paths):
        # 补全可能的扩展名
        candidates = [repo_root / tp]
        if not any(tp.endswith(ext) for ext in [".cpp", ".h", ".cmake", ".md"]):
            candidates.append(repo_root / f"{tp}.cpp")
            candidates.append(repo_root / f"{tp}.h")
            candidates.append(repo_root / f"{tp}.cmake")
            candidates.append(repo_root / f"{tp}.md")
        
        found = any(c.exists() for c in candidates)
        if not found:
            missing_paths.append(tp)

    if missing_paths:
        for mp in missing_paths[:5]:  # 最多报5个
            result.add_warning(f"证据测试文件在代码库中未找到对应路径: {mp}")
        if len(missing_paths) > 5:
            result.add_warning(f"... 共有 {len(missing_paths)} 个测试证据路径未在本地直接匹配")

def lint_file(filepath: Path, repo_root: Path, verify_paths: bool = True) -> LintResult:
    result = LintResult(str(filepath))
    try:
        content = filepath.read_text(encoding="utf-8-sig")
    except Exception as e:
        result.add_error(f"读取文件失败: {e}")
        return result

    # 1. Frontmatter 检查
    frontmatter, body = parse_frontmatter(content)
    if not frontmatter:
        result.add_error("缺失 YAML Frontmatter 元数据头部 (--- ... ---)")
    else:
        for field in REQUIRED_FRONTMATTER_FIELDS:
            if field not in frontmatter:
                result.add_warning(f"Frontmatter 缺失推荐字段: '{field}'")

    # 2. 一级标题检查
    h1_match = re.search(r'^#\s+(.+)$', body, re.MULTILINE)
    if not h1_match:
        result.add_error("缺少文档一级标题 (# <Module> 算法登记)")
    else:
        h1_text = h1_match.group(1).strip()
        if "算法" not in h1_text:
            result.add_warning(f"一级标题 '{h1_text}' 建议包含 '算法登记' 或 '算法'")

    # 3. 算法登记表检查
    check_algorithms_table(body, result)

    # 4. 算法详述要素检查
    check_algorithm_sections(body, result)

    # 5. 测试路径校验
    if verify_paths:
        check_evidence_paths(body, repo_root, result)

    # 6. 非目标检查
    if not re.search(r'非目标|刻意不实现|boundaries\.md', body):
        result.add_info("建议补充 [非目标] 章节或链接至 boundaries.md")

    return result

def print_result(result: LintResult):
    status_icon = "PASS" if result.is_valid and not result.warnings else ("WARN" if result.is_valid else "FAIL")
    print(f"[{status_icon}] {result.filepath}")
    if result.algorithms_found:
        print(f"       登记算法数量: {len(result.algorithms_found)}")
    for err in result.errors:
        print(f"       [ERROR] {err}")
    for warn in result.warnings:
        print(f"       [WARN]  {warn}")
    for inf in result.info:
        print(f"       [INFO]  {inf}")

def main():
    parser = argparse.ArgumentParser(description="1Q algorithms.md 规范检查工具")
    parser.add_argument("files", nargs="*", help="要检查的 algorithms.md 文件列表")
    parser.add_argument("--all", action="store_true", help="扫描整个仓库中的 docs/*/algorithms.md")
    parser.add_argument("--repo-root", default=".", help="仓库根目录 (默认当前目录)")
    parser.add_argument("--no-path-check", action="store_true", help="跳过对本地测试文件路径真实性的检测")
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    target_files = []

    if args.all:
        docs_dir = repo_root / "docs"
        if not docs_dir.exists() and (repo_root / "1q" / "docs").exists():
            docs_dir = repo_root / "1q" / "docs"
            repo_root = repo_root / "1q"
        
        target_files = sorted(list(docs_dir.glob("*/algorithms.md")))
    elif args.files:
        target_files = [Path(f).resolve() for f in args.files]
    else:
        # 默认检查当前或 1q/docs
        candidate_docs = repo_root / "docs"
        if not candidate_docs.exists() and (repo_root / "1q" / "docs").exists():
            candidate_docs = repo_root / "1q" / "docs"
            repo_root = repo_root / "1q"
        if candidate_docs.exists():
            target_files = sorted(list(candidate_docs.glob("*/algorithms.md")))

    if not target_files:
        print("未找到需要检查的 algorithms.md 文件。")
        sys.exit(0)

    print(f"=== 开始 1Q 算法文档规范检查 (共 {len(target_files)} 个文件) ===")
    total_errors = 0
    total_warnings = 0

    for tf in target_files:
        res = lint_file(tf, repo_root, verify_paths=not args.no_path_check)
        print_result(res)
        total_errors += len(res.errors)
        total_warnings += len(res.warnings)
        print("-" * 60)

    print(f"=== 检查完成: 错误 {total_errors} 项, 告警 {total_warnings} 项 ===")
    if total_errors > 0:
        sys.exit(1)
    sys.exit(0)

if __name__ == "__main__":
    main()
