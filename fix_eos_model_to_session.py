import os

repo_root = "/Users/aurora/Code/1q"

replacements = [
    ("model::EosCycleInput", "session::EosCycleInput"),
    ("model::EosTargetState", "session::EosTargetState"),
    ("model::EosTargetStateList", "session::EosTargetStateList"),
    ("model::DayNightType", "session::DayNightType"),
    ("model::EosCycleResult", "session::EosCycleResult"),
    ("model::ValidateEosCycleInput", "session::ValidateEosCycleInput"),
]

for root, _, files in os.walk(repo_root):
    if "build" in root or ".git" in root or "tools" in root:
        continue
    for file in files:
        if file.endswith((".h", ".hpp", ".cpp", ".cc", ".md", ".fbs")):
            filepath = os.path.join(root, file)
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
            
            orig = content
            for old, new in replacements:
                content = content.replace(old, new)
                
            if orig != content:
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(content)
                print(f"Updated {filepath}")

