#!/bin/sh
# Raw CAN traffic must never be sent over the mesh (plan 26.5, issue #32).
# Code that talks to MeshCore (src/telemetry, src/node) may only see the
# vehicle snapshot and DTC decoding, never frames or the OBD transport.
cd "$(dirname "$0")/.." || exit 1
bad=$(grep -rnE '#include +["<](can/|obd/obd_manager|obd/can_bus)|CanFrame|CanBus|ObdManager' src/telemetry src/node)
if [ -n "$bad" ]; then
  echo "MeshCore-facing code references the CAN/OBD transport:"
  echo "$bad"
  exit 1
fi
echo "ok: src/telemetry and src/node do not touch CAN frames"
