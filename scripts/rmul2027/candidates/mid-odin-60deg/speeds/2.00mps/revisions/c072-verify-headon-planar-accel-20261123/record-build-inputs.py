#!/usr/bin/env python3
"""Record or verify the exact source, parameters and libraries for an offline round."""
import argparse
import hashlib
import json
from pathlib import Path

LIBRARIES = (
    "gicp-install/small_gicp_relocalization/lib/libsmall_gicp_relocalization.so",
    "planner-install/rmul_c072_planner/lib/libc072_navigation_plugins.so",
    "planner-install/sensor_scan_generation/lib/libsensor_scan_generation.so",
    "planner-install/nav2_velocity_smoother/lib/libvelocity_smoother_core.so",
)


def hash_inputs(root):
    sources = sorted(
        path.relative_to(root).as_posix()
        for path in (root / "source").rglob("*")
        if path.is_file() and "__pycache__" not in path.parts
    )
    return {
        name: hashlib.sha256((root / name).read_bytes()).hexdigest()
        for name in ("params.yaml", *sources, *LIBRARIES)
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--verify", action="store_true")
    options = parser.parse_args()
    manifest = options.root / "build-inputs.json"
    current = hash_inputs(options.root)
    if options.verify:
        expected = json.loads(manifest.read_text())
        if current != expected:
            changed = sorted(
                key for key in current.keys() | expected.keys()
                if current.get(key) != expected.get(key)
            )
            raise SystemExit("Build inputs changed: " + ", ".join(changed))
    else:
        manifest.write_text(json.dumps(current, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
