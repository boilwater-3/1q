import os, glob, re, fnmatch

def regex_replace_in_file(path, replacements):
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()
    new_content = content
    for pattern, repl in replacements:
        new_content = re.sub(pattern, repl, new_content)
    if new_content != content:
        with open(path, "w", encoding="utf-8") as f:
            f.write(new_content)
        print(f"Updated {path}")

files = []
for root, dirs, filenames in os.walk("/Users/aurora/Code/1q/"):
    if ".git" in root or "build" in root: continue
    for ext in ["*.h", "*.cpp", "*.cmake", "*.fbs"]:
        for filename in fnmatch.filter(filenames, ext):
            files.append(os.path.join(root, filename))

esr_replacements = [
    (r"\bObservationQuality\b", "EsrObservationQuality"),
    (r"\bThreatLevel\b", "EsrThreatLevel"),
    (r"\bEmitterMode\b", "EsrEmitterMode")
]

eos_replacements = [
    (r"session::EosWorkMode", "config::EosWorkMode"),
    (r"eos::session::EosWorkMode", "eos::config::EosWorkMode"),
    (r"eos_session::EosWorkMode", "eos_config::EosWorkMode"),
    (r"session::DayNightType", "model::DayNightType"),
    (r"eos::session::DayNightType", "eos::model::DayNightType")
]

for f in files:
    if "electronic_surveillance_radar" in f or "esr" in f.lower() or "tests" in f or "examples" in f:
        regex_replace_in_file(f, esr_replacements)
    if "electro_optical_sensor" in f or "eos" in f.lower() or "tests" in f or "examples" in f or "CMakeLists" in f:
        regex_replace_in_file(f, eos_replacements)
