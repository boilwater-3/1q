---
name: completeness-check-before-stop
enabled: false
event: stop
conditions:
  - field: transcript
    operator: contains
    pattern: (invoke name="Edit"|invoke name="Write")
  - field: transcript
    operator: not_contains
    pattern: (completeness-review|code-review|simplify|ctest --preset|gtest_filter)
action: warn
---

⚠️ **Completeness review recommended before stopping**

No review or test activity detected in this session. Before stopping:

1. **Check plan coverage**: run `/completeness-review` to verify which plan items are done, partial, or missing
2. **Run tests**: `ctest --preset llvm-ninja-debug-local --output-on-failure`
3. **Code quality**: run `/simplify` to improve clarity and reduce redundancy
4. **Verify Done Means**: check each item in CLAUDE.md's Done Means checklist

Ignore this reminder if the session was read-only (no code changes).
