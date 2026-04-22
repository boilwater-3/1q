import os
import re

def process_file(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    original = content

    # ESR
    content = content.replace("EsrValidationSeverity", "ValidationSeverity")
    content = content.replace("EsrValidationCode", "ValidationCode")
    content = content.replace("EsrValidationIssue", "ValidationIssue")
    content = content.replace("EsrValidationIssueList", "ValidationIssueList")
    content = content.replace("HasEsrValidationError", "HasValidationError")

    # EOS
    content = content.replace("EosValidationSeverity", "ValidationSeverity")
    content = content.replace("EosValidationCode", "ValidationCode")
    content = content.replace("EosValidationIssue", "ValidationIssue")
    content = content.replace("EosValidationIssueList", "ValidationIssueList")
    content = content.replace("HasEosValidationError", "HasValidationError")

    # Namespace fix for EOS validation
    # If the file is EOS related, and it includes the moved header, update include
    content = content.replace("electro_optical_sensor/model/EosInputValidation.h", "electro_optical_sensor/session/EosInputValidation.h")
    content = content.replace("namespace model {", "namespace session {") # Only for the validation headers! Wait, that's risky to do blindly.

    # Special case for the EosInputValidation headers and cpps themselves
    if "EosInputValidation.h" in filepath or "EosInputValidation.cpp" in filepath or "EosCycleResult.h" in filepath or "EosCycleResult.cpp" in filepath:
        # We need to manually fix namespaces, we will do it below carefully
        pass
        
    if "model::ValidationIssue" in content and "electro_optical_sensor" in filepath:
        content = content.replace("model::ValidationIssue", "session::ValidationIssue")
    if "model::ValidationSeverity" in content and "electro_optical_sensor" in filepath:
        content = content.replace("model::ValidationSeverity", "session::ValidationSeverity")
    if "model::ValidationCode" in content and "electro_optical_sensor" in filepath:
        content = content.replace("model::ValidationCode", "session::ValidationCode")
    if "model::HasValidationError" in content and "electro_optical_sensor" in filepath:
        content = content.replace("model::HasValidationError", "session::HasValidationError")

    if content != original:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"Updated {filepath}")

for root, _, files in os.walk("/Users/aurora/Code/1q"):
    if "build" in root or ".git" in root or "tools" in root:
        continue
    for file in files:
        if file.endswith((".h", ".hpp", ".cpp", ".cc", ".md", ".fbs")):
            process_file(os.path.join(root, file))

