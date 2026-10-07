#!/usr/bin/env bash
# Turn a `pio run` log into a markdown table of RAM/flash use per environment.
# Usage: scripts/size_report.sh build.log   (output suits $GITHUB_STEP_SUMMARY)
set -euo pipefail

log="${1:?usage: size_report.sh <pio-run-log>}"

echo "### Firmware size"
echo
echo "| env | RAM | flash |"
echo "|---|---|---|"
awk '
  /^Processing / { env = $2 }
  /^RAM:/        { sub(/^RAM: +\[[^]]*\] +/, ""); ram[env] = $0 }
  /^Flash:/      { sub(/^Flash: +\[[^]]*\] +/, ""); fl[env] = $0; if (!(env in seen)) { seen[env] = 1; order[++n] = env } }
  END { for (i = 1; i <= n; i++) printf "| %s | %s | %s |\n", order[i], ram[order[i]], fl[order[i]] }
' "$log"
