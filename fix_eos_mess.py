import os

repo_root = "/Users/aurora/Code/1q"

replacements = [
    ("eos_::electro_optical_sensor::session", "eos_session"),
    ("namespace eos_model = ::electro_optical_sensor::model;", ""),
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

