#!/usr/bin/env python3
# OpenRoad Dynamics — plugin packaging.
# SPDX-License-Identifier: MIT
"""Make plugin/OpenRoadDynamics self-contained so it can be copied into a project.

Copies (standard library only):
  src/core/include          -> plugin/OpenRoadDynamics/Source/OpenRoadCore/Core/include
  src/core/src              -> plugin/OpenRoadDynamics/Source/OpenRoadCore/Core/src
  LICENSE                   -> plugin/OpenRoadDynamics/LICENSE
  THIRD_PARTY_NOTICES.md    -> plugin/OpenRoadDynamics/THIRD_PARTY_NOTICES.md

and verifies that Private/OrdCoreAmalgamation.cpp includes every core source.

Usage:
  python3 tools/package_plugin.py            # package
  python3 tools/package_plugin.py --check    # verify only (amalgamation list, packaged copy up to date)
  python3 tools/package_plugin.py --clean    # remove the packaged copies
"""

from __future__ import annotations

import argparse
import filecmp
import re
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CORE = REPO / "src" / "core"
PLUGIN = REPO / "plugin" / "OpenRoadDynamics"
PACKAGED_CORE = PLUGIN / "Source" / "OpenRoadCore" / "Core"
AMALGAMATION = PLUGIN / "Source" / "OpenRoadDynamics" / "Private" / "OrdCoreAmalgamation.cpp"
ROOT_FILES = ("LICENSE", "THIRD_PARTY_NOTICES.md")
COPIED_SUFFIXES = {".hpp", ".h", ".cpp", ".inl"}


def core_sources() -> list[str]:
    return sorted(p.name for p in (CORE / "src").glob("*.cpp"))


def amalgamated_sources() -> list[str]:
    text = AMALGAMATION.read_text(encoding="utf-8")
    return sorted(re.findall(r'^\s*#\s*include\s+"src/([^"]+\.cpp)"', text, flags=re.MULTILINE))


def check_amalgamation() -> list[str]:
    errors = []
    have, want = set(amalgamated_sources()), set(core_sources())
    for name in sorted(want - have):
        errors.append(f"{AMALGAMATION.relative_to(REPO)} is missing #include \"src/{name}\"")
    for name in sorted(have - want):
        errors.append(f"{AMALGAMATION.relative_to(REPO)} includes src/{name}, which no longer exists in src/core/src")
    return errors


def source_files(root: Path) -> dict[str, Path]:
    return {
        str(p.relative_to(root)): p
        for p in sorted(root.rglob("*"))
        if p.is_file() and p.suffix in COPIED_SUFFIXES
    }


def check_packaged() -> list[str]:
    if not PACKAGED_CORE.exists():
        return []  # not packaged: the build uses the repository path
    errors = []
    for sub in ("include", "src"):
        src, dst = source_files(CORE / sub), source_files(PACKAGED_CORE / sub)
        for rel in sorted(set(src) | set(dst)):
            if rel not in dst:
                errors.append(f"packaged core is missing {sub}/{rel}")
            elif rel not in src:
                errors.append(f"packaged core has stale {sub}/{rel}")
            elif not filecmp.cmp(src[rel], dst[rel], shallow=False):
                errors.append(f"packaged core {sub}/{rel} is out of date")
    for name in ROOT_FILES:
        copy = PLUGIN / name
        if not copy.exists() or not filecmp.cmp(REPO / name, copy, shallow=False):
            errors.append(f"plugin {name} is missing or out of date")
    return errors


def clean() -> None:
    if PACKAGED_CORE.exists():
        shutil.rmtree(PACKAGED_CORE)
    for name in ROOT_FILES:
        (PLUGIN / name).unlink(missing_ok=True)


def package() -> None:
    clean()
    for sub in ("include", "src"):
        shutil.copytree(CORE / sub, PACKAGED_CORE / sub,
                        ignore=lambda d, names: [n for n in names
                                                 if (Path(d) / n).is_file() and Path(n).suffix not in COPIED_SUFFIXES])
    for name in ROOT_FILES:
        shutil.copy2(REPO / name, PLUGIN / name)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="verify only; exit 1 on problems")
    mode.add_argument("--clean", action="store_true", help="remove the packaged copies")
    args = parser.parse_args()

    for required in (CORE / "include" / "ord" / "vehicle.hpp", AMALGAMATION, *(REPO / n for n in ROOT_FILES)):
        if not required.exists():
            print(f"error: {required} not found (run from an OpenRoad Dynamics checkout)", file=sys.stderr)
            return 1

    if args.clean:
        clean()
        print(f"removed packaged core from {PLUGIN.relative_to(REPO)}")
        return 0

    errors = check_amalgamation()
    if args.check:
        errors += check_packaged()
    if errors:
        for e in errors:
            print(f"error: {e}", file=sys.stderr)
        return 1
    if args.check:
        print("ok: amalgamation lists every core source" +
              ("; packaged core is up to date" if PACKAGED_CORE.exists() else "; plugin uses the repository core"))
        return 0

    package()
    n = len(source_files(PACKAGED_CORE))
    print(f"packaged {n} core files into {PACKAGED_CORE.relative_to(REPO)} (+ {', '.join(ROOT_FILES)})")
    print("copy plugin/OpenRoadDynamics into <YourProject>/Plugins/ to use it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
