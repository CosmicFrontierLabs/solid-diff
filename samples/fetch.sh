#!/usr/bin/env bash
# Fetch public sample SLDPRT files (all SolidWorks 2015+ container format).
# Sources: ros/solidworks_urdf_exporter (MIT) and xarial/codestack examples.
set -euo pipefail
cd "$(dirname "$0")"

sw2urdf=https://raw.githubusercontent.com/ros/solidworks_urdf_exporter/master/examples
codestack=https://raw.githubusercontent.com/xarial/codestack/master/solidworks-api

curl -sfLO "$sw2urdf/3_DOF_ARM/3_DOF_ARM_BASE.SLDPRT"
curl -sfLO "$sw2urdf/4_WHEELER/4_WHEELER_WHEEL.SLDPRT"
curl -sfLO "$codestack/geometry/precise-bounding-box/bbox-precision.SLDPRT"
curl -sfLO "$codestack/document/macro-feature/multi-extrude/MacroFeatureMultiExtrude.SLDPRT"

ls -la ./*.SLDPRT
