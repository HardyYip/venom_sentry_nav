#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/native-nav"
round="nav-$(date +%Y%m%d-%H%M%S-%N)"
./isolated.bash bash /trial/contact-check.bash "$round-contact"
python3 run-round.py "$round"
python3 audit.py "$round" "$round-contact"
./isolated.bash bash /trial/speed-track/run-track.bash speed-track straight "$round-lane"
./isolated.bash bash /trial/speed-track/run-ramp.bash speed-track straight "$round-ramp"
python3 verify-speed.py "$round-lane" "$round-ramp"
./isolated.bash bash /trial/speed-track/run-track.bash speed-track diagonal "$round-diagonal"
python3 speed-track/verify-diagonal.py "$round-diagonal"
printf '结果目录 %s/%s、%s-lane、%s-ramp、%s-diagonal\n' "$PWD" "$round" "$round" "$round" "$round"
