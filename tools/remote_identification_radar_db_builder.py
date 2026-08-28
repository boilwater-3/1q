#!/usr/bin/env python3
"""识别特征数据库建库工具（schema v1.1，JSON → SQLite）。

用法：
  python3 tools/remote_identification_radar_db_builder.py \\
      --input examples/basic_config/remote_identification_radar/recognition_database_input.json \\
      --output examples/basic_config/remote_identification_radar/target_feature_database_v1.1.db

输入为设计文档 §7.3 格式 JSON（含 units/display_name/created_utc/aspect 区间）；
输出为通过加载器同级校验的 SQLite 库文件。DDL 读取唯一事实源
schemas/remote_identification_radar/recognition_feature_database.sql（与 C++ 加载器/测试共用，
禁止在本工具内维护第二份 DDL）。

校验规则与 RecognitionFeatureDatabase::Load 一致：meta 六键必填、units 七量纲
必填且 rcs=='dBsm'、id 非空唯一、prior>0、引用存在、aspect min<=max、
模板 std>0（mean 可缺省为 0）。仅 stdlib 依赖。
"""

import argparse
import json
import sqlite3
import sys
from pathlib import Path

SCHEMA_PATH = (
    Path(__file__).resolve().parent.parent
    / "schemas" / "remote_identification_radar" / "recognition_feature_database.sql"
)

REQUIRED_UNITS = [
    "rcs", "speed", "altitude", "acceleration", "turn_radius", "polarization", "range",
]

# 模板组表：JSON 组键 → (列前缀, std 错误字段名) 列表。
# mean/std 列名 = prefix + "_mean"/"_std"（turn_radius 与 rcs 特殊）。
TEMPLATE_GROUPS = {
    "rcs": {
        "table": "rcs_templates",
        "mean": ("mean_dbsm", "mean_dbsm"),
        "std": ("std_db", "std_db"),
        "extra": [
            ("azimuth_variation_db", "azimuth_variation_db"),
            ("elevation_variation_db", "elevation_variation_db"),
            ("minimum_aspect_coverage_deg", "minimum_aspect_coverage_deg"),
        ],
    },
    "motion": {
        "table": "motion_templates",
        "features": [
            (("speed_mps", "mean"), ("speed_mps", "std"), "speed"),
            (("altitude_m", "mean"), ("altitude_m", "std"), "altitude"),
            (("acceleration_mps2", "mean"), ("acceleration_mps2", "std"), "acceleration"),
            (("turn_radius_m", "mean_log10"), ("turn_radius_m", "std_log10"), "turn_radius"),
        ],
    },
    "polarization": {
        "table": "polarization_templates",
        "features": [
            (("energy_difference_db", "mean"), ("energy_difference_db", "std"),
             "energy_difference"),
            (("relative_difference_db", "mean"), ("relative_difference_db", "std"),
             "relative_difference"),
            (("energy_sum_db", "mean"), ("energy_sum_db", "std"), "energy_sum"),
        ],
    },
    "range_profile": {
        "table": "range_profile_templates",
        "features": [
            (("length_m", "mean"), ("length_m", "std"), "length"),
            (("peak_count", "mean"), ("peak_count", "std"), "peak_count"),
            (("peak_energy_concentration", "mean"),
             ("peak_energy_concentration", "std"), "peak_energy_concentration"),
        ],
        "extra": [("minimum_bandwidth_hz", "minimum_bandwidth_hz")],
    },
}


def _nonempty(value, label, errors):
    """字符串非空校验；返回清洗后的值或 None。"""
    if value is None or str(value) == "":
        errors.append("%s must be a non-empty string" % label)
        return None
    return str(value)


def _positive_float(value, label, errors, default=None):
    """数值 > 0 校验；None 且无默认则报错。"""
    if value is None:
        if default is not None:
            return default
        errors.append("%s must be > 0" % label)
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        errors.append("%s must be a number" % label)
        return None
    if not value > 0:
        errors.append("%s must be > 0" % label)
        return None
    return value


def _optional_float(value, label, errors, default=None):
    """可空数值校验（负数不拒绝，保持加载器语义）。"""
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        errors.append("%s must be a number" % label)
        return None


def _aspect_range(profile, key, errors):
    """aspect 区间 [min, max]；缺省全范围。均存在时校验 min <= max。"""
    value = profile.get(key)
    if value is None:
        return None, None
    if not isinstance(value, list) or len(value) != 2:
        errors.append("applicability.%s must be a [min, max] pair" % key)
        return None, None
    lo, hi = value
    try:
        lo, hi = float(lo), float(hi)
    except (TypeError, ValueError):
        errors.append("applicability.%s must be numbers" % key)
        return None, None
    if lo > hi:
        errors.append("applicability.%s: min must be <= max" % key)
        return None, None
    return lo, hi


def validate(doc, errors):
    """按加载器同级规则校验文档；错误追加到 errors。"""
    meta = doc.get("meta", {})
    _nonempty(meta.get("schema_version"), "meta.schema_version", errors)
    if meta.get("schema_version") != "1.1":
        errors.append("meta.schema_version must be '1.1'")
    _nonempty(meta.get("database_id"), "meta.database_id", errors)
    _nonempty(meta.get("version"), "meta.version", errors)
    _nonempty(meta.get("created_utc"), "meta.created_utc", errors)
    channels = meta.get("polarization_channels")
    if not channels:
        errors.append("meta.polarization_channels must be a non-empty list")
    _nonempty(meta.get("polarization_energy_reference"),
              "meta.polarization_energy_reference", errors)

    units = doc.get("units", {})
    for quantity in REQUIRED_UNITS:
        unit = units.get(quantity)
        _nonempty(unit, "units.%s" % quantity, errors)
    if units.get("rcs") != "dBsm":
        errors.append("units.rcs must be 'dBsm'")

    category_ids = set()
    for category in doc.get("categories", []):
        cid = _nonempty(category.get("category_id"), "categories[].category_id", errors)
        if cid in category_ids:
            errors.append("categories[%s].category_id must be unique" % cid)
        category_ids.add(cid)
        _positive_float(category.get("prior"), "categories[%s].prior" % cid, errors)

    model_ids = set()
    for model in doc.get("models", []):
        mid = _nonempty(model.get("model_id"), "models[].model_id", errors)
        if mid in model_ids:
            errors.append("models[%s].model_id must be unique" % mid)
        model_ids.add(mid)
        cat = _nonempty(model.get("category_id"), "models[%s].category_id" % mid, errors)
        if cat is not None and cat not in category_ids:
            errors.append("models[%s].category_id references unknown category '%s'"
                          % (mid, cat))
        _positive_float(model.get("prior"), "models[%s].prior" % mid, errors)

        profile_ids = set()
        for profile in model.get("profiles", []):
            pid = _nonempty(profile.get("profile_id"),
                            "models[%s].profiles[].profile_id" % mid, errors)
            if pid in profile_ids:
                errors.append("profiles[%s].profile_id must be unique within the model" % pid)
            profile_ids.add(pid)
            applicability = profile.get("applicability", {})
            _optional_float(applicability.get("min_snr_db"),
                            "profiles[%s].applicability.min_snr_db" % pid, errors)
            _optional_float(applicability.get("max_range_resolution_m"),
                            "profiles[%s].applicability.max_range_resolution_m" % pid, errors)
            _aspect_range(applicability, "aspect_az_deg", errors)
            _aspect_range(applicability, "aspect_el_deg", errors)
            _validate_templates(profile, pid, errors)


def _nested(data, keys):
    """按键链取嵌套值；任一层缺失返回 None。"""
    for key in keys:
        if not isinstance(data, dict) or key not in data:
            return None
        data = data[key]
    return data


def _validate_templates(profile, pid, errors):
    """模板组：组存在时 std 必填 > 0；mean 可缺省。"""
    for group_key, group in TEMPLATE_GROUPS.items():
        data = profile.get(group_key)
        if data is None:
            continue
        label = "profiles[%s].%s" % (pid, group_key)
        if group_key == "rcs":
            _positive_float(data.get("std_db"), "%s.std_db" % label, errors)
            for key, column in group["extra"]:
                _optional_float(data.get(key), "%s.%s" % (label, key), errors)
        else:
            for mean_key, std_key, prefix in group["features"]:
                std_value = _nested(data, std_key)
                if std_value is None:
                    # std 必填（行存在即子模板存在）。
                    errors.append("%s.%s must be > 0" % (label, ".".join(std_key)))
                    continue
                _positive_float(std_value, "%s.%s" % (label, ".".join(std_key)), errors)
                _optional_float(_nested(data, mean_key), "%s.%s" % (label, ".".join(mean_key)),
                                errors)
            for key, column in group.get("extra", []):
                _optional_float(data.get(key), "%s.%s" % (label, key), errors)


def _meta_rows(doc):
    meta = doc["meta"]
    rows = [
        ("schema_version", str(meta["schema_version"])),
        ("database_id", str(meta["database_id"])),
        ("version", str(meta["version"])),
        ("created_utc", str(meta["created_utc"])),
        ("polarization_channels", ",".join(meta["polarization_channels"])),
        ("polarization_energy_reference", str(meta["polarization_energy_reference"])),
    ]
    return rows


def _template_row(model_id, profile_id, group_key, data):
    """模板组行；缺组返回 None。"""
    if data is None:
        return None
    group = TEMPLATE_GROUPS[group_key]
    if group_key == "rcs":
        return (profile_id, model_id,
                _optional_float(data.get("mean_dbsm"), "", []), float(data["std_db"]),
                _optional_float(data.get("azimuth_variation_db"), "", []),
                _optional_float(data.get("elevation_variation_db"), "", []),
                _optional_float(data.get("minimum_aspect_coverage_deg"), "", []))
    row = [profile_id, model_id]
    for mean_key, std_key, prefix in group["features"]:
        row.append(_optional_float(_nested(data, mean_key), "", []))
        row.append(float(_nested(data, std_key)))
    for key, column in group.get("extra", []):
        row.append(_optional_float(data.get(key), "", []))
    return tuple(row)


def build(doc, output_path):
    """校验后写入 SQLite 库；失败抛 ValueError（含全部错误）。"""
    errors = []
    validate(doc, errors)
    if errors:
        raise ValueError("\n".join(errors))

    if not SCHEMA_PATH.is_file():
        raise ValueError("missing schema file: %s" % SCHEMA_PATH)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        output_path.unlink()

    conn = sqlite3.connect(str(output_path))
    try:
        conn.executescript(SCHEMA_PATH.read_text(encoding="utf-8"))
        conn.executemany("INSERT INTO meta VALUES (?, ?)", _meta_rows(doc))
        conn.executemany("INSERT INTO units VALUES (?, ?)",
                         [(q, doc["units"][q]) for q in REQUIRED_UNITS])
        for category in doc.get("categories", []):
            conn.execute("INSERT INTO categories VALUES (?, ?, ?)",
                         (category["category_id"], category.get("display_name"),
                          category["prior"]))
        for model in doc.get("models", []):
            conn.execute("INSERT INTO models VALUES (?, ?, ?, ?)",
                         (model["model_id"], model["category_id"],
                          model.get("display_name"), model["prior"]))
            for profile in model.get("profiles", []):
                applicability = profile.get("applicability", {})
                az_min, az_max = _aspect_range(applicability, "aspect_az_deg", [])
                el_min, el_max = _aspect_range(applicability, "aspect_el_deg", [])
                conn.execute(
                    "INSERT INTO profiles VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                    (profile["profile_id"], model["model_id"],
                     applicability.get("min_snr_db"), applicability.get("max_range_resolution_m"),
                     az_min, az_max, el_min, el_max))
                for group_key in TEMPLATE_GROUPS:
                    row = _template_row(model["model_id"], profile["profile_id"],
                                        group_key, profile.get(group_key))
                    if row is None:
                        continue
                    table = TEMPLATE_GROUPS[group_key]["table"]
                    placeholders = ",".join("?" * len(row))
                    conn.execute("INSERT INTO %s VALUES (%s)" % (table, placeholders), row)
        conn.commit()
    finally:
        conn.close()


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="识别基线 JSON 输入文件")
    parser.add_argument("--output", required=True, help="SQLite 输出文件路径")
    args = parser.parse_args(argv)

    try:
        with open(args.input, encoding="utf-8") as handle:
            doc = json.load(handle)
    except (OSError, ValueError) as exc:
        print("error: cannot read input %s: %s" % (args.input, exc), file=sys.stderr)
        return 1
    try:
        build(doc, args.output)
    except ValueError as exc:
        print("validation failed:\n%s" % exc, file=sys.stderr)
        return 1
    print("wrote %s" % args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
