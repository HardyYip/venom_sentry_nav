"""Check the recorded library search order against the compiled simulation overlays."""
import hashlib
import json
from pathlib import Path
import re
import sys

root = Path(sys.argv[1]).resolve()
workspace = Path("/tmp/rmul-original-20261119/workspace")
expected = json.loads((root / "tested-inputs.json").read_text())
libraries = [
    key for key in expected if key.endswith((
        "/libsmall_gicp_relocalization.so", "/libc072_navigation_plugins.so",
        "/libsensor_scan_generation.so", "/libvelocity_smoother_core.so",
    ))
]
assert len(libraries) == 4
evidence = []
for role in ("fixed-left", "fixed-reverse-right", "moving-standard"):
    for launcher in sorted((root / role / "cases").glob("*/launcher.log")):
        match = re.search(r"Logs: (.+)", launcher.read_text())
        if not match:
            continue
        manifest = workspace / "logs" / Path(match[1]).name / "tf2-runtime.json"
        dirs = json.loads(manifest.read_text())["navigation_ld_library_path"].split(":")
        row = {"stage": role, "case": launcher.parent.name, "libraries": {}}
        for key in libraries:
            library = Path(key).name
            for item in dirs:
                runtime_path = Path(item)
                if Path("/evidence") in runtime_path.parents:
                    local = root.parent / runtime_path.relative_to("/evidence")
                elif Path("/home/hardy/pb2025_nav_ws") in runtime_path.parents:
                    local = workspace / runtime_path.relative_to("/home/hardy/pb2025_nav_ws")
                else:
                    continue
                if (local / library).is_file():
                    digest = hashlib.sha256((local / library).read_bytes()).hexdigest()
                    if digest != expected[key]:
                        raise RuntimeError(f"{launcher.parent.name} loaded search prefix for different {library}")
                    row["libraries"][library] = {"prefix": item, "sha256": digest}
                    break
            else:
                raise RuntimeError(f"No workspace or evidence prefix provides {library}")
        evidence.append(row)
if len(evidence) != 15:
    raise SystemExit(f"Only {len(evidence)} formal launches available")
(root / "runtime-libraries.json").write_text(json.dumps(evidence, indent=2) + "\n")
