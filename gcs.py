# gcs.py
"""Writers for global coordinate system reference files."""
from typing import List

from rotors_xml import RotorCfg


def _f(x: float) -> str:
    return f"{x:.10f}"


def _angle_list(n: int, cw: bool) -> List[float]:
    if n <= 0:
        return [0.0]
    step = 360.0 / n
    sign = -1.0 if cw else 1.0
    return [sign * k * step for k in range(n)]


def write_gcs_refs(rotors: List[RotorCfg], out_path: str, add_blade_bases: bool = True) -> None:
    """
    GCS.ref with:
      - ROTOR_i at each rotor center
      - ROTOR_i_blade1_BASEk, k=1..blade_count, rotated about Z by eulr(0,0,angle*degrad)
        angle sign: CCW positive, CW negative
    """
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# GCS.ref\n")
        for r in rotors:
            x, y, z = r.center_xyz
            f.write(f"reference: ROTOR_{r.index}, #gen ref\n")
            f.write(f"    reference, global, {_f(x)}, {_f(y)}, {_f(z)},\n")
            f.write("    reference, global, eye,\n")
            f.write("    reference, global, null,\n")
            f.write("    reference, global, null;\n\n")

            if add_blade_bases:
                cw = ("顺时" in (r.direction or "")) or ((r.direction or "").strip().lower() == "cw")
                angles = _angle_list(max(1, r.blade_count), cw)
                for k, ang in enumerate(angles, start=1):
                    f.write(f"reference: ROTOR_{r.index}_blade1_BASE{k}, #第{k}片\n")
                    f.write(
                        f"    reference, ROTOR_{r.index}, 0.0000000000, 0.0000000000, 0.0000000000,\n"
                    )
                    f.write(
                        f"    reference, ROTOR_{r.index}, eulr,0,0,{ang:.10f}*degrad,\n"
                    )
                    f.write(f"    reference, ROTOR_{r.index}, null,\n")
                    f.write(f"    reference, ROTOR_{r.index}, null;\n\n")
