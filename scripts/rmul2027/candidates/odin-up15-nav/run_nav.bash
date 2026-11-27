#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/native-nav"
round="nav-$(date +%Y%m%d-%H%M%S-%N)"
python3 run-round.py "$round"
python3 audit.py "$round"
printf '结果保存在 %s/%s/audit.json\n' "$PWD" "$round"
