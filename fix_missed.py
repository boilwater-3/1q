import os

repo_root = "/Users/aurora/Code/1q"

replacements = [
    ("model::ValidationIssue", "session::ValidationIssue"),
    ("model::ValidationCode", "session::ValidationCode"),
    ("model::ValidationSeverity", "session::ValidationSeverity"),
    ("model::HasValidationError", "session::HasValidationError"),
    ("session::EosCycleResult", "::electro_optical_sensor::session::EosCycleResult"),
    ("session::EosCycleInput", "::electro_optical_sensor::session::EosCycleInput"),
    ("session::DayNightType", "::electro_optical_sensor::session::DayNightType"),
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
                
            # Quick hack to fix double qualified names just in case
            content = content.replace("::electro_optical_sensor::::electro_optical_sensor", "::electro_optical_sensor")
            content = content.replace("electro_optical_sensor::::electro_optical_sensor", "electro_optical_sensor")
            content = content.replace("::electro_optical_sensor::session::ValidateEosCycleInput", "session::ValidateEosCycleInput")
            
            if orig != content:
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(content)
                print(f"Updated {filepath}")

