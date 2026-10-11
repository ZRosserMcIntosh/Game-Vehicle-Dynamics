#!/bin/sh
# OpenRoad Dynamics — headless check of the UE adapter's physics-thread code.
# SPDX-License-Identifier: MIT
# Compiles the adapter's engine-free parts against ue_stubs/ (NOT Unreal Engine)
# together with the core, then runs main.cpp. Usage: ./run.sh [compiler]
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../../.." && pwd)
CXX=${1:-${CXX:-c++}}
OUT=${TMPDIR:-/tmp}/ord_ue_headless_check
PRIV="$HERE/../../Source/OpenRoadDynamics/Private"
"$CXX" -std=c++20 -O2 -Wall -Wextra -Wshadow -Werror -fno-exceptions -fno-rtti \
  -I"$HERE/ue_stubs" -I"$PRIV" -I"$REPO/src/core/include" -I"$REPO/src" \
  "$HERE/main.cpp" "$PRIV/OrdVehicleSim.cpp" "$REPO"/src/core/src/*.cpp "$REPO/src/vehicles/reference_vehicles.cpp" \
  -o "$OUT"
"$OUT"
