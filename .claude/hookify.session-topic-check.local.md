---
name: session-topic-check
enabled: false
event: prompt
conditions:
  - field: user_prompt
    operator: not_contains
    pattern: (topic|feature/|branch|plan|review|discuss|read.only)
action: warn
---

💡 **Tip**: Are you on `main`? Consider entering plan mode first — the plan topic will become the branch name (`feature/<topic-slug>`). Skip if read-only.
