#!/usr/bin/env python3
"""PreToolUse hook: intercept git commit, analyze scope, recommend proportional review.

Receives PreToolUse hook JSON on stdin. For git commit commands, analyzes staged
changes and returns a scoped decision:
  - Trivial (config/docs only):    allow silently
  - Minor  (<3 C++ files, <50 lines): allow with light reminder
  - Major  (>=3 C++ files or >=50 lines): ask — prompt agent to review first
"""

import json
import subprocess
import sys
import os

def analyze_staged_changes():
    """Analyze git diff --cached and return (tier, details)."""
    try:
        result = subprocess.run(
            ["git", "diff", "--cached", "--stat"],
            capture_output=True, text=True, timeout=5
        )
        stat_output = result.stdout.strip()
        if not stat_output:
            return "empty", {}

        lines = stat_output.split("\n")
        # Last line: "N files changed, M insertions(+), K deletions(-)"
        summary = lines[-1] if lines else ""

        cpp_patterns = (".cpp", ".h", ".hpp", ".cc", ".cxx", ".c", ".cmake")
        cpp_files = 0
        cpp_insertions = 0
        cpp_deletions = 0
        total_files = 0

        for line in lines[:-1]:  # skip summary line
            parts = line.strip().split()
            if not parts:
                continue
            total_files += 1
            filename = parts[0] if len(parts) > 1 else parts[0]
            if filename.endswith(cpp_patterns):
                cpp_files += 1
                try:
                    # Format: "file | N +++---" or "file | Bin 0 -> N bytes"
                    changes = line.split("|")[-1].strip() if "|" in line else ""
                    if "+" in changes or "-" in changes:
                        plus = changes.count("+")
                        minus = changes.count("-")
                        cpp_insertions += plus
                        cpp_deletions += minus
                except (ValueError, IndexError):
                    pass

        # Determine changed modules
        modules = set()
        for line in lines[:-1]:
            parts = line.strip().split()
            if not parts:
                continue
            path = parts[0] if len(parts) > 1 else parts[0]
            if path.startswith("src/"):
                # Extract module: src/<module>/
                parts = path.split("/")
                if len(parts) >= 3:
                    modules.add(parts[1])
            elif path.startswith("include/1q/"):
                parts = path.split("/")
                if len(parts) >= 3:
                    modules.add(parts[2])
            elif path.startswith("tests/"):
                parts = path.split("/")
                if len(parts) >= 3:
                    modules.add(parts[2])

        cpp_total = cpp_insertions + cpp_deletions

        details = {
            "cpp_files": cpp_files,
            "cpp_lines": cpp_total,
            "total_files": total_files,
            "modules": sorted(modules),
        }

        if cpp_files == 0:
            return "trivial", details
        elif cpp_files < 3 and cpp_total < 50:
            return "minor", details
        else:
            return "major", details

    except Exception as e:
        # If analysis fails, err on the safe side — allow but note the error
        return "error", {"error": str(e)}


def main():
    try:
        hook_input = json.load(sys.stdin)
    except (json.JSONDecodeError, Exception):
        # Can't parse input — allow silently
        print(json.dumps({"decision": "allow"}))
        return

    tool_name = hook_input.get("tool_name", "")
    tool_input = hook_input.get("tool_input", {})
    command = tool_input.get("command", "")

    # Only intercept git commit commands
    if tool_name != "Bash":
        print(json.dumps({"decision": "allow"}))
        return

    if not command.strip().startswith("git commit"):
        print(json.dumps({"decision": "allow"}))
        return

    # Skip non-standard commits (amend, fixup, etc.) — let them through
    cmd_tokens = command.strip().split()
    if any(t in cmd_tokens for t in ["--amend", "fixup", "squash"]):
        print(json.dumps({"decision": "allow"}))
        return

    tier, details = analyze_staged_changes()

    if tier == "empty":
        print(json.dumps({"decision": "allow"}))
        return

    if tier == "trivial":
        # Config/docs only — no review needed
        print(json.dumps({
            "decision": "allow",
            "systemMessage": "Pre-commit: config/docs only — review skipped."
        }))
        return

    if tier == "minor":
        mods = ", ".join(details.get("modules", [])) or "unknown"
        print(json.dumps({
            "decision": "allow",
            "systemMessage": (
                f"Pre-commit: light change ({details['cpp_files']} C++ files, "
                f"~{details['cpp_lines']} lines, modules: {mods}). "
                "Self-review recommended but full /completeness-review not required."
            )
        }))
        return

    if tier == "major":
        mods = ", ".join(details.get("modules", [])) or "unknown"
        print(json.dumps({
            "decision": "ask",
            "reason": (
                f"⚠️ Significant change detected: {details['cpp_files']} C++ files, "
                f"~{details['cpp_lines']} lines across [{mods}].\n\n"
                "Run /completeness-review before committing?\n\n"
                "- Yes: agent will run the review first, then commit\n"
                "- No: proceed with commit as-is (not recommended for untested changes)"
            )
        }))
        return

    if tier == "error":
        # Analysis failed — allow but note
        print(json.dumps({
            "decision": "allow",
            "systemMessage": (
                "Pre-commit: unable to analyze change scope "
                f"({details.get('error', 'unknown error')}). Proceeding without review check."
            )
        }))
        return

    print(json.dumps({"decision": "allow"}))


if __name__ == "__main__":
    main()
