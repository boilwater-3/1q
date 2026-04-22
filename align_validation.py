import os
import glob
import shutil

repo_root = "/Users/aurora/Code/1q"

def replace_in_file(filepath, replacements):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
        
    orig = content
    for old, new in replacements:
        content = content.replace(old, new)
        
    if orig != content:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"Updated {filepath}")

# 1. Rename types across all files
type_replacements = [
    ("EsrValidationSeverity", "ValidationSeverity"),
    ("EsrValidationCode", "ValidationCode"),
    ("EsrValidationIssue", "ValidationIssue"),
    ("EsrValidationIssueList", "ValidationIssueList"),
    ("HasEsrValidationError", "HasValidationError"),
    
    ("EosValidationSeverity", "ValidationSeverity"),
    ("EosValidationCode", "ValidationCode"),
    ("EosValidationIssue", "ValidationIssue"),
    ("EosValidationIssueList", "ValidationIssueList"),
    ("HasEosValidationError", "HasValidationError"),
    
    ("electro_optical_sensor/model/EosInputValidation.h", "electro_optical_sensor/session/EosInputValidation.h"),
    ("electro_optical_sensor/model/EosCycleInput.h", "electro_optical_sensor/session/EosCycleInput.h"),
    ("electro_optical_sensor/model/EosCycleResult.h", "electro_optical_sensor/session/EosCycleResult.h"),
]

for root, _, files in os.walk(repo_root):
    if "build" in root or ".git" in root or "tools" in root:
        continue
    for file in files:
        if file.endswith((".h", ".hpp", ".cpp", ".cc", ".md", ".fbs")):
            replace_in_file(os.path.join(root, file), type_replacements)

# 2. Move EOS files to session namespace
eos_files_to_move = [
    ("include/1q/electro_optical_sensor/model/EosInputValidation.h", "include/1q/electro_optical_sensor/session/EosInputValidation.h"),
    ("src/electro_optical_sensor/model/EosInputValidation.cpp", "src/electro_optical_sensor/session/EosInputValidation.cpp"),
    ("include/1q/electro_optical_sensor/model/EosCycleInput.h", "include/1q/electro_optical_sensor/session/EosCycleInput.h"),
    ("src/electro_optical_sensor/model/EosCycleInput.cpp", "src/electro_optical_sensor/session/EosCycleInput.cpp"),
    ("include/1q/electro_optical_sensor/model/EosCycleResult.h", "include/1q/electro_optical_sensor/session/EosCycleResult.h"),
    ("src/electro_optical_sensor/model/EosCycleResult.cpp", "src/electro_optical_sensor/session/EosCycleResult.cpp"),
]

for old_rel, new_rel in eos_files_to_move:
    old_path = os.path.join(repo_root, old_rel)
    new_path = os.path.join(repo_root, new_rel)
    if os.path.exists(old_path):
        os.rename(old_path, new_path)
        print(f"Moved {old_rel} to {new_rel}")
        
        # Also need to update the namespace inside these specific files
        with open(new_path, 'r', encoding='utf-8') as f:
            content = f.read()
        content = content.replace("namespace model {", "namespace session {")
        content = content.replace("}  // namespace model", "}  // namespace session")
        with open(new_path, 'w', encoding='utf-8') as f:
            f.write(content)

# 3. Update namespace usages in EOS code that referred to model::EosCycleInput, model::ValidateEosCycleInput, etc.
# But since we only renamed the types above, there might be 'model::ValidationIssue' or 'model::EosCycleInput'.
eos_namespace_replacements = [
    ("model::ValidationIssue", "session::ValidationIssue"),
    ("model::ValidationCode", "session::ValidationCode"),
    ("model::ValidationSeverity", "session::ValidationSeverity"),
    ("model::ValidationIssueList", "session::ValidationIssueList"),
    ("model::HasValidationError", "session::HasValidationError"),
    ("model::ValidateEosCycleInput", "session::ValidateEosCycleInput"),
    ("model::EosCycleInput", "session::EosCycleInput"),
    ("model::EosCycleResult", "session::EosCycleResult"),
    ("model::EosDayNightProfile", "session::EosDayNightProfile"), # wait, what about DayNightType? DayNightType is in EosCycleInput.h?
]

# Let's check DayNightType. It was moved to model in Phase 1. 
# We should probably keep DayNightType in model? But it's defined in EosCycleInput.h!
# If EosCycleInput.h moves to session, DayNightType moves to session too!

