# gcs.py
"""Writers for global coordinate system reference files."""
from typing import List

from rotors_xml import RotorCfg


def _f(value: float) -> str:
    return f"{value:.10f}"


def write_gcs_refs(rotors: List[RotorCfg], out_path: str) -> None:
    """Write a GCS.ref file enumerating each rotor origin."""
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write("# GCS.ref\n")
        for rotor in rotors:
            x, y, z = rotor.center_xyz
            fh.write(f"reference: ROTOR_{rotor.index}, #gen ref\n")
            fh.write(f"    reference, global, {_f(x)}, {_f(y)}, {_f(z)},\n")
            fh.write("    reference, global, eye,\n")
            fh.write("    reference, global, null,\n")
            fh.write("    reference, global, null;\n\n")
