#!/usr/bin/env bash
# Fetch public sample SLDPRT files.
# Sources: ros/solidworks_urdf_exporter (MIT) and xarial/codestack examples.
# All are SolidWorks 2015+ container format except the two marked legacy.
set -euo pipefail
cd "$(dirname "$0")"

sw2urdf=https://raw.githubusercontent.com/ros/solidworks_urdf_exporter/master/examples
codestack=https://raw.githubusercontent.com/xarial/codestack/master/solidworks-api

curl -sfLO "$sw2urdf/3_DOF_ARM/3_DOF_ARM_BASE.SLDPRT"
curl -sfLO "$sw2urdf/3_DOF_ARM/3_DOF_ARM_END_EFFECTOR.SLDPRT"
curl -sfLO "$sw2urdf/3_DOF_ARM/3_DOF_ARM_SEGMENT.SLDPRT"
curl -sfLO "$sw2urdf/4_WHEELER/4_WHEELER_CHASSIS.SLDPRT"
curl -sfLO "$sw2urdf/4_WHEELER/4_WHEELER_WHEEL.SLDPRT"
curl -sfLO "$sw2urdf/ORIGINAL_3_DOF_ARM/Arm_base.SLDPRT"
curl -sfLO "$sw2urdf/ORIGINAL_3_DOF_ARM/Arm_brace.SLDPRT"
curl -sfLO "$sw2urdf/ORIGINAL_3_DOF_ARM/Arm_link_tube.SLDPRT"
curl -sfL "$sw2urdf/ORIGINAL_3_DOF_ARM/Skin%20Link.SLDPRT" -o Skin_Link.SLDPRT
curl -sfLO "$codestack/geometry/precise-bounding-box/bbox-precision.SLDPRT"
curl -sfLO "$codestack/document/macro-feature/multi-extrude/MacroFeatureMultiExtrude.SLDPRT"
curl -sfLO "$codestack/document/selection/api-only-selection/extrude-selection-example.SLDPRT"
curl -sfLO "$codestack/document/tracking-objects/tracking-ids/tracking-ids-sample.SLDPRT"
curl -sfLO "$codestack/getting-started/scripts/power-shell/model-generator/template.SLDPRT"
# Legacy pre-2015 OLE2 container (out of scope for now; kept for future work):
curl -sfLO "$sw2urdf/ORIGINAL_3_DOF_ARM/Arm_link1_tube.SLDPRT"
curl -sfLO "$sw2urdf/TOY_BLOCK/BlockA.SLDPRT"

ls -la ./*.SLDPRT
