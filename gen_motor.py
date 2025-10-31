from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import List, Tuple

from io_utils import parse_triplet, rel_include_line, write_text


def generate(act_motor_xml: str, out_dir: str) -> Tuple[List[str], List[str]]:
    """Produce motor artifacts from act_motor.xml if available."""

    node_includes: List[str] = []
    element_includes: List[str] = []

    if not os.path.isfile(act_motor_xml):
        return node_includes, element_includes

    root = ET.parse(act_motor_xml).getroot()

    try:
        mass = float((root.findtext("质量") or "0").strip())
    except Exception:
        mass = 0.0

    x, y, z = parse_triplet(root.findtext("质心位置") or "0,0,0")
    rx, ry, rz = parse_triplet(root.findtext("方向") or "0,0,0")
    i1, i2, i3 = parse_triplet(root.findtext("转动惯量") or "0,0,0")

    nod_path = os.path.join(out_dir, "motor.nod")
    write_text(
        nod_path,
        "structural:  MOTOR, dynamic,\n"
        f"    reference, global, {x:.6f}, {y:.6f}, {z:.6f},\n"
        "    reference, global, eye,\n"
        "    reference, global, null,\n"
        "    reference, global, null;\n",
    )
    node_includes.append(rel_include_line(nod_path, start_dir=out_dir, indent=4))

    elm_path = os.path.join(out_dir, "motor.elm")
    write_text(
        elm_path,
        "body: MOTOR, MOTOR,\n"
        f"   {mass:.6g},\n"
        "    reference, node, null,\n"
        f"    reference, global, eulr, {rx:.6g}, {ry:.6g}, {rz:.6g},\n"
        f"        diag, {i1:.6g}, {i2:.6g}, {i3:.6g};\n",
    )
    element_includes.append(rel_include_line(elm_path, start_dir=out_dir, indent=4))

    return node_includes, element_includes
