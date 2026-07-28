#!/usr/bin/env python3
"""Analyze staged changes for review scope classification.

Used by git pre-commit hook and manual review decisions.
Reads git diff --cached and outputs a JSON tier classification:

  {"tier": "trivial", "cpp_files": 0, "cpp_lines": 0, "modules": []}
  {"tier": "minor",   "cpp_files": N, "cpp_lines": M, "modules": [...]}
  {"tier": "major",   "cpp_files": N, "cpp_lines": M, "modules": [...]}
  {"tier": "empty"}
  {"tier": "error", "error": "..."}

Tiers:
  trivial — no C++ files changed (config/docs only)
  minor   — <3 C++ files and <50 lines
  major   — >=3 C++ files or >=50 lines
"""

import json
import subprocess
import sys


def analyze():
    try:
        result = subprocess.run(
            ["git", "diff", "--cached", "--stat"],
            capture_output=True, text=True, timeout=5
        )
        stat_output = result.stdout.strip()
        if not stat_output:
            return {"tier": "empty"}

        lines = stat_output.split("\n")

        cpp_patterns = (".cpp", ".h", ".hpp", ".cc", ".cxx", ".c", ".cmake")
        cpp_files = 0
        cpp_insertions = 0
        cpp_deletions = 0
        total_files = 0

        for line in lines[:-1]:
            parts = line.strip().split()
            if not parts:
                continue
            total_files += 1
            filename = parts[0] if len(parts) > 1 else parts[0]
            if filename.endswith(cpp_patterns):
                cpp_files += 1
                try:
                    changes = line.split("|")[-1].strip() if "|" in line else ""
                    if "+" in changes or "-" in changes:
                        cpp_insertions += changes.count("+")
                        cpp_deletions += changes.count("-")
                except (ValueError, IndexError):
                    pass

        modules = set()
        for line in lines[:-1]:
            parts = line.strip().split()
            if not parts:
                continue
            path = parts[0] if len(parts) > 1 else parts[0]
            if path.startswith("src/"):
                path_parts = path.split("/")
                if len(path_parts) >= 3:
                    modules.add(path_parts[1])
            elif path.startswith("include/1q/"):
                path_parts = path.split("/")
                if len(path_parts) >= 3:
                    modules.add(path_parts[2])
            elif path.startswith("tests/"):
                path_parts = path.split("/")
                if len(path_parts) >= 3:
                    modules.add(path_parts[2])

        cpp_total = cpp_insertions + cpp_deletions

        if cpp_files == 0:
            return {"tier": "trivial", "cpp_files": 0, "cpp_lines": 0,
                    "total_files": total_files, "modules": sorted(modules)}
        elif cpp_files < 3 and cpp_total < 50:
            return {"tier": "minor", "cpp_files": cpp_files, "cpp_lines": cpp_total,
                    "total_files": total_files, "modules": sorted(modules)}
        else:
            return {"tier": "major", "cpp_files": cpp_files, "cpp_lines": cpp_total,
                    "total_files": total_files, "modules": sorted(modules)}

    except Exception as e:
        return {"tier": "error", "error": str(e)}


if __name__ == "__main__":
    print(json.dumps(analyze()))
