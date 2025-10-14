# refs_nodes.py
"""Utilities to write blade reference frames and structural nodes."""

import math
from io_blade import TipData
from grid import BladeGrid


def _f(val: float) -> str:
    return f"{val:.10f}"


def write_refs(tip: TipData, grid: BladeGrid, name: str, out_path: str, y_sign: float = 1.0) -> None:
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.ref\n")
        f.write(f"# name={name}\n")
        for i, x in enumerate(grid.nodes, start=1):
            twist_deg = grid.interp_tip("ROTAPI_deg", x)
            c = math.cos(math.radians(twist_deg)); s = math.sin(math.radians(twist_deg))

            f.write(f"reference: CURR_ROTOR + CURR_blade + FEATH + {i}, #gen ref\n")
            f.write(f"    reference, CURR_ROTOR + BASE, {_f(x)}, 0., 0.,\n")
            f.write("    reference, CURR_ROTOR + BASE,\n")
            f.write("        1, 1., 0., 0.,\n")
            f.write(f"        2, 0., {_f(c)}, {_f(s)},\n")
            f.write("    reference, CURR_ROTOR + BASE, null,\n")
            f.write("    reference, CURR_ROTOR + BASE, null;\n\n")

            YNA = y_sign * grid.interp_tip("YNA", x)
            ZNA = grid.interp_tip("ZNA", x)
            f.write(f"reference: CURR_ROTOR + CURR_blade + NEUTR + {i}, #gen ref\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, 0., {_f(YNA)}, {_f(ZNA)},\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, eye,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, null,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, null;\n\n")

            YCG = y_sign * grid.interp_tip("YCG", x)
            ZCG = grid.interp_tip("ZCG", x)
            f.write(f"reference: CURR_ROTOR + CURR_blade + BODY + {i}, #gen ref\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, 0., {_f(YCG)}, {_f(ZCG)},\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, eye,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, null,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, null;\n\n")


def write_nodes(grid: BladeGrid, name: str, out_path: str) -> None:
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.nod\n")
        f.write(f"# name={name}\n")
        for i, _x in enumerate(grid.nodes, start=1):
            f.write(f"structural:  CURR_ROTOR + CURR_blade + {i}, dynamic,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + NEUTR + {i}, null,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + NEUTR + {i}, eye,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + NEUTR + {i}, null,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + NEUTR + {i}, null;\n\n")
