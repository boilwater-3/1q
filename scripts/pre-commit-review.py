#!/usr/bin/env python3
"""PreToolUse hook: intercept git commit, auto-create feature branch, scale review.

Receives PreToolUse hook JSON on stdin. For git commit commands:
  1. If on main/master, auto-create feature/<topic> branch from commit message
  2. Analyze staged changes, return scoped decision:
     - Trivial (config/docs only):    allow silently
     - Minor  (<3 C++ files, <50 lines): allow with light reminder
     - Major  (>=3 C++ files or >=50 lines): ask — prompt agent to review first
"""

import json
import re
import subprocess
import sys


def current_branch():
    """Return the current git branch name, or empty string on failure."""
    try:
        r = subprocess.run(
            ["git", "branch", "--show-current"],
            capture_output=True, text=True, timeout=3
        )
        return r.stdout.strip()
    except Exception:
        return ""


def extract_topic(commit_command):
    """Extract a kebab-case topic slug from the commit message subject line."""
    msg = ""
    # Try to extract from -m "..." or -m '...' or --message "..."
    m = re.search(r'(?:-m|--message)\s+["\']([^"\']+)', commit_command)
    if m:
        msg = m.group(1)
    else:
        # Multi-line -m or no parseable message
        return None

    # Strip conventional commit prefix: type(scope): or type:
    msg = re.sub(r'^[a-z]+(\([^)]*\))?\s*:\s*', '', msg.strip())

    # Convert to kebab-case
    slug = msg.lower()
    slug = re.sub(r'[^a-z0-9\s-]', '', slug)
    slug = re.sub(r'\s+', '-', slug)
    slug = re.sub(r'-{2,}', '-', slug)
    slug = slug.strip('-')

    if not slug:
        return None

    # Truncate to reasonable length
    if len(slug) > 50:
        slug = slug[:50].rstrip('-')

    return f"feature/{slug}"


def ensure_feature_branch(commit_command):
    """If on main/master, create and switch to a feature branch.

    Returns (switched: bool, branch_name: str | None, error: str | None).
    """
    branch = current_branch()
    if branch not in ("main", "master"):
        return False, branch, None

    topic = extract_topic(commit_command)
    if not topic:
        topic = "feature/auto"

    # If branch already exists, append a number
    base = topic
    counter = 2
    while True:
        try:
            r = subprocess.run(
                ["git", "rev-parse", "--verify", topic],
                capture_output=True, timeout=3
            )
            if r.returncode == 0:
                topic = f"{base}-{counter}"
                counter += 1
            else:
                break
        except Exception:
            break

    try:
        subprocess.run(
            ["git", "checkout", "-b", topic],
            capture_output=True, text=True, timeout=5, check=True
        )
        return True, topic, None
    except subprocess.CalledProcessError as e:
        return False, None, f"git checkout -b {topic} failed: {e.stderr.strip()}"


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
                    changes = line.split("|")[-1].strip() if "|" in line else ""
                    if "+" in changes or "-" in changes:
                        cpp_insertions += changes.count("+")
                        cpp_deletions += changes.count("-")
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
        return "error", {"error": str(e)}


def main():
    try:
        hook_input = json.load(sys.stdin)
    except (json.JSONDecodeError, Exception):
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

    # Skip non-standard commits
    cmd_tokens = command.strip().split()
    if any(t in cmd_tokens for t in ["--amend", "fixup", "squash"]):
        print(json.dumps({"decision": "allow"}))
        return

    # ── Step 1: Auto-create feature branch if on main ──
    switched, branch_name, branch_error = ensure_feature_branch(command)
    if branch_error:
        print(json.dumps({
            "decision": "allow",
            "reason": (
                f"Pre-commit: could not auto-create feature branch: {branch_error}"
            )
        }))
        return

    branch_msg = ""
    if switched and branch_name:
        branch_msg = f"Auto-created branch `{branch_name}`. "
    elif branch_name and branch_name not in ("main", "master"):
        branch_msg = f"On branch `{branch_name}`. "

    # ── Step 2: Analyze scope and decide review tier ──
    tier, details = analyze_staged_changes()

    if tier == "empty":
        print(json.dumps({"decision": "allow"}))
        return

    if tier == "trivial":
        print(json.dumps({
            "decision": "allow",
            "reason": (
                f"Pre-commit: {branch_msg}config/docs only — review skipped."
            )
        }))
        return

    if tier == "minor":
        mods = ", ".join(details.get("modules", [])) or "unknown"
        print(json.dumps({
            "decision": "allow",
            "reason": (
                f"Pre-commit: {branch_msg}light change ({details['cpp_files']} C++ files, "
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
                f"{branch_msg}"
                f"⚠️ Significant change: {details['cpp_files']} C++ files, "
                f"~{details['cpp_lines']} lines across [{mods}].\n\n"
                "Run /completeness-review before committing?\n\n"
                "- Yes: agent will run the review first, then commit\n"
                "- No: proceed with commit as-is"
            )
        }))
        return

    if tier == "error":
        print(json.dumps({
            "decision": "allow",
            "reason": (
                f"Pre-commit: {branch_msg}unable to analyze scope "
                f"({details.get('error', 'unknown error')}). Proceeding."
            )
        }))
        return

    print(json.dumps({"decision": "allow"}))


if __name__ == "__main__":
    main()
