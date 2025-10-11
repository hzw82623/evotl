# refs_nodes.py
"""
Write reference frames (blade.ref) and structural nodes (blade.nod).

Reference systems per node i (index starting at 1 in files):
- FEATH+i: position x=nodes[i], rotation about x by ROTAPI_deg(nodes[i])
- NEUTR+i: w.r.t FEATH+i, translation (0, YNA, ZNA); orientation same as FEATH
- BODY+i:  w.r.t FEATH+i, translation (0, YCG, ZCG); orientation same as FEATH

Nodes:
- One dynamic structural node per grid node, attached to NEUTR+i (position/orientation taken from NEUTR+i).
"""

import math
from typing import Optional
from io_blade import TipData
from grid import BladeGrid

# number formatting
def _f(val: float) -> str:
    return f"{val:.10f}"

def write_refs(tip: TipData, grid: BladeGrid, name: str, out_path: str) -> None:
    """
    Write blade.ref including FEATH/NEUTR/BODY for each node.

    File format (MBDyn-like line style)
    -----------------------------------
    reference: CURR_ROTOR + CURR_{name} + FEATH + {i}, #gen ref
        reference, CURR_ROTOR + BASE, {x}, 0., 0.,
        reference, CURR_ROTOR + BASE,
            1, 1., 0., 0.,
            2, 0., cos, sin,
        reference, CURR_ROTOR + BASE, null,
        reference, CURR_ROTOR + BASE, null;

    reference: CURR_ROTOR + CURR_{name} + NEUTR + {i}, #gen ref
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, 0., YNA, ZNA,
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, eye,
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, null,
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, null;

    reference: CURR_ROTOR + CURR_{name} + BODY + {i}, #gen ref
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, 0., YCG, ZCG,
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, eye,
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, null,
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, null;
    """
    if grid.interp_tip is None:
        raise RuntimeError("grid.interp_tip is not attached. Call attach_interpolators() first.")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.ref\n")
        f.write(f"# name={name}\n")
        for i, x in enumerate(grid.nodes, start=1):
            # FEATH: position x, rotation about x by ROTAPI_deg
            twist_deg = grid.interp_tip("ROTAPI_deg", x)
            c = math.cos(math.radians(twist_deg))
            s = math.sin(math.radians(twist_deg))

            f.write(f"reference: CURR_ROTOR + CURR_{name} + FEATH + {i}, #gen ref\n")
            f.write(f"    reference, CURR_ROTOR + BASE, {_f(x)}, 0., 0.,\n")
            f.write(f"    reference, CURR_ROTOR + BASE,\n")
            f.write(f"        1, 1., 0., 0.,\n")
            f.write(f"        2, 0., {_f(c)}, {_f(s)},\n")
            f.write(f"    reference, CURR_ROTOR + BASE, null,\n")
            f.write(f"    reference, CURR_ROTOR + BASE, null;\n\n")

            # NEUTR: translate (0, YNA, ZNA) from FEATH; same orientation (eye)
            YNA = grid.interp_tip("YNA", x)
            ZNA = grid.interp_tip("ZNA", x)
            f.write(f"reference: CURR_ROTOR + CURR_{name} + NEUTR + {i}, #gen ref\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, 0., {_f(YNA)}, {_f(ZNA)},\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, eye,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, null,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, null;\n\n")

            # BODY: translate (0, YCG, ZCG) from FEATH; same orientation (eye)
            YCG = grid.interp_tip("YCG", x)
            ZCG = grid.interp_tip("ZCG", x)
            f.write(f"reference: CURR_ROTOR + CURR_{name} + BODY + {i}, #gen ref\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, 0., {_f(YCG)}, {_f(ZCG)},\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, eye,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, null,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, null;\n\n")


def write_nodes(grid: BladeGrid, name: str, out_path: str) -> None:
    """
    Write blade.nod: dynamic nodes placed at NEUTR+i.

    File format (MBDyn-like line style)
    -----------------------------------
    structural:  CURR_ROTOR + CURR_{name} + {i}, dynamic,
        reference, CURR_ROTOR + CURR_{name} + NEUTR + {i}, null,
        reference, CURR_ROTOR + CURR_{name} + NEUTR + {i}, eye,
        reference, CURR_ROTOR + CURR_{name} + NEUTR + {i}, null,
        reference, CURR_ROTOR + CURR_{name} + NEUTR + {i}, null;
    """
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.nod\n")
        f.write(f"# name={name}\n")
        for i, _x in enumerate(grid.nodes, start=1):
            f.write(f"structural:  CURR_ROTOR + CURR_{name} + {i}, dynamic,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + NEUTR + {i}, null,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + NEUTR + {i}, eye,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + NEUTR + {i}, null,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + NEUTR + {i}, null;\n\n")
