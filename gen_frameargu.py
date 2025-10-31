from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import List, Tuple

from femgen_mb import generate_fem
from io_utils import parse_triplet, rel_include_line, write_text


def generate(aer_frameargu_xml: str, out_dir: str) -> Tuple[List[str], List[str]]:
    """Produce frameargu artifacts and include lines when FEM inputs are valid."""

    node_includes: List[str] = []
    element_includes: List[str] = []

    if not os.path.isfile(aer_frameargu_xml):
        return node_includes, element_includes

    root = ET.parse(aer_frameargu_xml).getroot()
    jet = root.find("机身1")
    if jet is None:
        return node_includes, element_includes

    origin_text = (jet.findtext("原点位置") or "").strip()
    ox, oy, oz = parse_triplet(origin_text)

    bdf_path = (jet.findtext("结构模型文件") or "").strip()
    f06_path = (jet.findtext("模态结果文件") or "").strip()
    if not (bdf_path and f06_path and os.path.isfile(bdf_path) and os.path.isfile(f06_path)):
        return node_includes, element_includes

    fem_path = os.path.join(out_dir, "frameargu.fem")
    generate_fem(f06_path, bdf_path, fem_path)

    nod_path = os.path.join(out_dir, "frameargu.nod")
    write_text(
        nod_path,
        "structural:  FRAMEARGU, modal,\n"
        f"    reference, global, {ox:.6f}, {oy:.6f}, {oz:.6f},\n"
        "    reference, global, eye,\n"
        "    reference, global, null,\n"
        "    reference, global, null;\n",
    )
    node_includes.append(rel_include_line(nod_path, start_dir=out_dir, indent=4))

    elm_path = os.path.join(out_dir, "frameargu.elm")
    write_text(elm_path, rel_include_line(fem_path, start_dir=out_dir, indent=0) + "\n")
    element_includes.append(rel_include_line(elm_path, start_dir=out_dir, indent=4))

    return node_includes, element_includes
