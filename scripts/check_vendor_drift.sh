#!/bin/sh
# Fails if a MeshCore file that RoadNode shadows has changed since the RoadNode copy was last
# reconciled (issue 58). Run after a submodule bump. Usage:
#   scripts/check_vendor_drift.sh            check (CI runs this)
#   scripts/check_vendor_drift.sh --update   record the current vendor files as reconciled
cd "$(dirname "$0")/.." || exit 1
VENDOR=vendor/MeshCore
BASELINE=${VENDOR_BASELINE:-scripts/vendor_baseline.txt}

FILES="examples/simple_sensor/main.cpp src/node/main.cpp
variants/heltec_v4/target.h src/telemetry/variant/target.h
variants/heltec_v4/target.cpp src/telemetry/variant/target.cpp"

if [ ! -e "$VENDOR/.git" ]; then
  echo "$VENDOR is not checked out (git submodule update --init)"
  exit 1
fi

if [ "$1" = "--update" ]; then
  commit=$(git -C "$VENDOR" rev-parse --short HEAD)
  {
    grep '^#' "$BASELINE" | grep -v '^# MeshCore commit'
    echo "# MeshCore commit at last reconcile: $commit"
    echo "$FILES" | while read -r v ours; do
      echo "$(git hash-object "$VENDOR/$v") $v $ours"
    done
  } > "$BASELINE.new" && mv "$BASELINE.new" "$BASELINE"
  echo "baseline updated to MeshCore $commit"
  exit 0
fi

old=$(sed -n 's/^# MeshCore commit at last reconcile: //p' "$BASELINE")
bad=0
while read -r hash v ours; do
  case "$hash" in '#'*|'') continue ;; esac
  now=$(git hash-object "$VENDOR/$v" 2>/dev/null)
  if [ "$now" != "$hash" ]; then
    echo "DRIFT: $VENDOR/$v changed since $ours was reconciled."
    echo "  see: git -C $VENDOR diff ${old:-<old>} HEAD -- $v"
    bad=1
  fi
done < "$BASELINE"
if [ "$bad" = 1 ]; then
  echo "Port the changes into the RoadNode copy, then run scripts/check_vendor_drift.sh --update."
  exit 1
fi
echo "ok: shadowed MeshCore files unchanged since last reconcile"
