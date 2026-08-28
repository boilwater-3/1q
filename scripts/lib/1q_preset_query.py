#!/usr/bin/env python3
"""Resolve CMakePresets.json (+ optional CMakeUserPresets.json) for 1q.sh guards."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any


def _load_preset_files(root: Path) -> list[dict[str, Any]]:
    presets: list[dict[str, Any]] = []
    for name in ("CMakePresets.json", "CMakeUserPresets.json"):
        path = root / name
        if not path.is_file():
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        presets.extend(data.get("configurePresets", []))
    return presets


def _load_build_presets(root: Path) -> list[dict[str, Any]]:
    presets: list[dict[str, Any]] = []
    for name in ("CMakePresets.json", "CMakeUserPresets.json"):
        path = root / name
        if not path.is_file():
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        presets.extend(data.get("buildPresets", []))
    return presets


def _merge_configure_preset(
    by_name: dict[str, dict[str, Any]], name: str, seen: set[str] | None = None
) -> dict[str, Any]:
    if seen is None:
        seen = set()
    if name in seen:
        raise SystemExit(f"circular configure preset inheritance at {name}")
    seen.add(name)
    preset = dict(by_name[name])
    inherit = preset.pop("inherits", None)
    if inherit is None:
        return preset
    parent = _merge_configure_preset(by_name, inherit, seen)
    merged = dict(parent)
    merged.update(preset)
    parent_cache = dict(parent.get("cacheVariables", {}))
    child_cache = dict(preset.get("cacheVariables", {}))
    parent_cache.update(child_cache)
    if parent_cache:
        merged["cacheVariables"] = parent_cache
    return merged


def _binary_dir(root: Path, configure_preset: str, merged: dict[str, Any]) -> Path:
    template = merged.get("binaryDir", "${sourceDir}/build/${presetName}")
    resolved = (
        template.replace("${sourceDir}", str(root))
        .replace("${presetName}", configure_preset)
    )
    return Path(resolved)


def _resolve_field(preset_name: str, field: str, root: Path) -> str:
    configure_presets = _load_preset_files(root)
    by_name = {p["name"]: p for p in configure_presets if "name" in p}

    build_presets = _load_build_presets(root)
    build_by_name = {p["name"]: p for p in build_presets if "name" in p}

    configure_name = preset_name
    if preset_name in build_by_name:
        configure_name = build_by_name[preset_name]["configurePreset"]
    elif preset_name not in by_name:
        raise SystemExit(f"unknown preset: {preset_name}")

    merged = _merge_configure_preset(by_name, configure_name)
    if field == "configurePreset":
        return configure_name
    if field == "binaryDir":
        return str(_binary_dir(root, configure_name, merged))
    raise SystemExit(f"unknown field: {field}")


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: 1q_preset_query.py <build-preset|configure-preset> <field>", file=sys.stderr)
        print("  field: configurePreset | binaryDir", file=sys.stderr)
        return 2

    preset_name = sys.argv[1]
    field = sys.argv[2]
    root = Path(os.environ.get("ONEQ_ROOT", ".")).resolve()

    try:
        print(_resolve_field(preset_name, field, root))
    except SystemExit as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0



if __name__ == "__main__":
    raise SystemExit(main())
