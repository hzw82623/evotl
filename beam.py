# beam.py
"""
Write structural beam3 elements into blade.beam.

Per element over [x1, x2], with mid xm = 0.5*(x1+x2):
- Three nodes: x1, xm, x2
- Two evaluation points for constitutive matrices:
    x_ev1 = xm - 0.5/sqrt(3) * (x2 - x1)
    x_ev2 = xm + 0.5/sqrt(3) * (x2 - x1)

Constitutive 6x6 (same as your spec):
Let Y1 = YCT - YNA, Z1 = ZCT - ZNA at the evaluation point.
Let c = cos(ROTAN), s = sin(ROTAN) (deg->rad).
GAy = EA / (2*(1+nu)); GAz = EA / (2*(1+nu))

A22 = EJY*c^2 + EJZ*s^2 + (Z1^2)*EA
A33 = EJZ*c^2 + EJY*s^2 + (Y1^2)*EA
A23 = (EJY - EJZ)*c*s - Y1*Z1*EA

K =
[ EA,    0,     0,    0,   Z1*EA,  -Y1*EA
  0,    GAy,    0,    0,     0,       0
  0,     0,    GAz,   0,     0,       0
  0,     0,     0,    GJ,    0,       0
  Z1*EA, 0,     0,    0,   A22,     A23
 -Y1*EA, 0,     0,    0,    A23,    A33 ]
"""

import math
from typing import List, Tuple
from io_blade import TipData
from grid import BladeGrid

# number formatting
def _f(val: float) -> str:
    return f"{val:.10f}"

def _fe(val: float) -> str:
    return f"{val:.6e}"

def _assemble_K(EA: float, EJY: float, EJZ: float, GJ: float,
                Y1: float, Z1: float, rotan_deg: float, nu: float) -> List[List[float]]:
    """
    Assemble symmetric 6x6 constitutive matrix at one evaluation point.
    """
    c = math.cos(math.radians(rotan_deg))
    s = math.sin(math.radians(rotan_deg))

    GAy = EA / (2.0 * (1.0 + nu))
    GAz = EA / (2.0 * (1.0 + nu))

    A22 = EJY * (c*c) + EJZ * (s*s) + (Z1*Z1) * EA
    A33 = EJZ * (c*c) + EJY * (s*s) + (Y1*Y1) * EA
    A23 = (EJY - EJZ) * c * s - Y1 * Z1 * EA

    K11 = EA
    K22 = GAy
    K33 = GAz
    K44 = GJ
    K15 = Z1 * EA
    K16 = -Y1 * EA

    # Construct full symmetric matrix
    K = [
        [K11,   0.0,  0.0,  0.0,  K15,   K16],
        [0.0,   K22,  0.0,  0.0,  0.0,   0.0],
        [0.0,   0.0,  K33,  0.0,  0.0,   0.0],
        [0.0,   0.0,  0.0,  K44,  0.0,   0.0],
        [K15,   0.0,  0.0,  0.0,  A22,   A23],
        [K16,   0.0,  0.0,  0.0,  A23,   A33],
    ]
    return K

def _write_matrix_block(fh, K: List[List[float]], comment: str):
    """
    Emit a 'linear elastic generic, matr,' block with 6 rows × 6 cols.
    """
    fh.write(f"        linear elastic generic, matr,  {comment}\n")
    for i in range(6):
        row = ", ".join(_fe(K[i][j]) for j in range(6))
        end = ",\n" if i < 5 else ";\n"
        fh.write(f"            {row}{end}")

def write_beam_file(tip: TipData, grid: BladeGrid, name: str, nu: float, out_path: str) -> None:
    """
    Write blade.beam: one 'beam3:' block per element with three nodes and two 'linear elastic generic, matr' blocks.

    Node placement
    --------------
    Each beam node is defined 'from nodes' and located w.r.t the NEUTR reference via shear-center offset:
        position, reference, CURR_ROTOR + CURR_{name} + NEUTR + {node_id}, 0., (YCT-YNA), (ZCT-ZNA),
        orientation, reference, CURR_ROTOR + CURR_{name} + NEUTR + {node_id}, eye,

    Constitutive matrices
    ---------------------
    For each of the two evaluation points x_ev1/x_ev2, evaluate EA, EJY, EJZ, GJ, ROTAN_deg and offsets.
    Then assemble the symmetric 6x6 K above.
    """
    if grid.interp_tip is None:
        raise RuntimeError("grid.interp_tip is not attached. Call attach_interpolators() first.")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.beam\n")
        f.write(f"# name={name}\n")
        for eidx, ((x1, xm, x2), (x_ev1, x_ev2)) in enumerate(zip(grid.elements, grid.eval_points), start=1):
            # Node ids follow end–mid–end indexing: (2*i+1, 2*i+2, 2*i+3)
            n1 = 2*eidx - 1
            n2 = 2*eidx
            n3 = 2*eidx + 1

            # Shear-center offsets at the three node positions (w.r.t NEUTR => YCT-YNA, ZCT-ZNA)
            def yz_sc(xpos: float):
                yct = grid.interp_tip("YCT", xpos)
                zct = grid.interp_tip("ZCT", xpos)
                yna = grid.interp_tip("YNA", xpos)
                zna = grid.interp_tip("ZNA", xpos)
                return (yct - yna, zct - zna)

            y1_sc, z1_sc = yz_sc(x1)
            y2_sc, z2_sc = yz_sc(xm)
            y3_sc, z3_sc = yz_sc(x2)

            f.write(f"# element {eidx}: x1={_f(x1)} xm={_f(xm)} x2={_f(x2)}\n")
            f.write( "beam3:\n")
            f.write(f"    CURR_ROTOR + CURR_{name} + {eidx},\n")
            # Node 1
            f.write(f"    CURR_ROTOR + CURR_{name} + {n1}\n")
            f.write(f"        position, reference, CURR_ROTOR + CURR_{name} + NEUTR + {n1}, 0., {_f(y1_sc)}, {_f(z1_sc)},\n")
            f.write(f"        orientation, reference, CURR_ROTOR + CURR_{name} + NEUTR + {n1}, eye,\n")
            # Node 2
            f.write(f"    CURR_ROTOR + CURR_{name} + {n2}\n")
            f.write(f"        position, reference, CURR_ROTOR + CURR_{name} + NEUTR + {n2}, 0., {_f(y2_sc)}, {_f(z2_sc)},\n")
            f.write(f"        orientation, reference, CURR_ROTOR + CURR_{name} + NEUTR + {n2}, eye,\n")
            # Node 3
            f.write(f"    CURR_ROTOR + CURR_{name} + {n3}\n")
            f.write(f"        position, reference, CURR_ROTOR + CURR_{name} + NEUTR + {n3}, 0., {_f(y3_sc)}, {_f(z3_sc)},\n")
            f.write(f"        orientation, reference, CURR_ROTOR + CURR_{name} + NEUTR + {n3}, eye,\n")
            f.write( "        from nodes,\n")

            # ---- Constitutive matrix at two Gauss points ----
            def K_at(xev: float):
                EA   = grid.interp_tip("EA", xev)
                EJY  = grid.interp_tip("EJY", xev)
                EJZ  = grid.interp_tip("EJZ", xev)
                GJ   = grid.interp_tip("GJ", xev)
                rot  = grid.interp_tip("ROTAN_deg", xev)
                yct  = grid.interp_tip("YCT", xev)
                zct  = grid.interp_tip("ZCT", xev)
                yna  = grid.interp_tip("YNA", xev)
                zna  = grid.interp_tip("ZNA", xev)
                Y1   = yct - yna
                Z1   = zct - zna
                return _assemble_K(EA, EJY, EJZ, GJ, Y1, Z1, rot, nu)

            K1 = K_at(x_ev1)
            K2 = K_at(x_ev2)

            _write_matrix_block(f, K1, f"# sec I @ r={_f(x_ev1)}")
            _write_matrix_block(f, K2, f"# sec II @ r={_f(x_ev2)}")

            f.write("\n")  # spacing between elements
