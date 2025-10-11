# bodies.py
"""
Lump distributed mass/inertia per node segment into 'body:' entries (blade.body).

For node i at x_i, define half-interval [x_L, x_R]:
  x_L = (x_{i-1} + x_i)/2, x_R = (x_i + x_{i+1})/2
(First/last nodes clamp to end sections.)

Midpoint rule approximation (endpoint mean):
  dL  = x_R - x_L
  M   = mean(dM at x_L,x_R) * dL
  JX  = mean(dJX at x_L,x_R) * dL
  JY  = mean(dJY at x_L,x_R) * dL + M/12 * dL^2
  JZ  = mean(dJZ at x_L,x_R) * dL + M/12 * dL^2

Attach body i to BODY+i reference system (i.e., zero offset/orientation here).
"""

from typing import List, Tuple
import math

from io_blade import TipData
from grid import BladeGrid

# number formatting
def _fe(val: float) -> str:
    return f"{val:.6e}"
def _f(val: float) -> str:
    return f"{val:.10f}"

def _mean2(a: float, b: float) -> float:
    return 0.5 * (a + b)

def write_bodies_file(tip: TipData, grid: BladeGrid, name: str, out_path: str) -> None:
    """
    Write blade.body: one 'body:' per grid node.

    File format (line style)
    ------------------------
    body: CURR_ROTOR + CURR_{name} + {i}, CURR_ROTOR + CURR_{name} + {i}
        ,
        {M},  # mass
        reference, CURR_ROTOR + CURR_{name} + BODY + {i}, 0., 0., 0.,
        reference, CURR_ROTOR + CURR_{name} + BODY + {i},
            diag, {JX}, {JY}, {JZ}
    ;
    """
    if grid.interp_tip is None:
        raise RuntimeError("grid.interp_tip is not attached. Call attach_interpolators() first.")

    nodes = grid.nodes
    sections = grid.sections
    N = len(nodes)
    if N == 0:
        raise ValueError("Grid has no nodes.")
    if len(sections) < 2:
        raise ValueError("Grid has fewer than 2 control sections.")

    total_mass = 0.0
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.body\n")
        f.write(f"# name={name}\n")

        for i, xi in enumerate(nodes, start=1):
            # half-interval bounds
            if i == 1:
                xL = sections[0]          # clamp left
                xR = 0.5 * (nodes[0] + nodes[1]) if N > 1 else sections[-1]
            elif i == N:
                xL = 0.5 * (nodes[N-2] + nodes[N-1]) if N > 1 else sections[0]
                xR = sections[-1]         # clamp right
            else:
                xL = 0.5 * (nodes[i-2] + nodes[i-1])
                xR = 0.5 * (nodes[i-1] + nodes[i])

            dL = max(0.0, xR - xL)
            if dL <= 0.0:
                # degenerate; write zero body to keep indexing consistent
                M = JX = JY = JZ = 0.0
            else:
                # endpoint mean values
                dM_L  = grid.interp_tip("dM",  xL)
                dM_R  = grid.interp_tip("dM",  xR)
                dJX_L = grid.interp_tip("dJX", xL)
                dJX_R = grid.interp_tip("dJX", xR)
                dJY_L = grid.interp_tip("dJY", xL)
                dJY_R = grid.interp_tip("dJY", xR)
                dJZ_L = grid.interp_tip("dJZ", xL)
                dJZ_R = grid.interp_tip("dJZ", xR)

                M  = max(0.0, _mean2(dM_L,  dM_R)  * dL)
                JX = max(0.0, _mean2(dJX_L, dJX_R) * dL)
                # parallel-axis + rod segment inertia about node-centered frame
                rod = (M * dL * dL) / 12.0
                JY = max(0.0, _mean2(dJY_L, dJY_R) * dL + rod)
                JZ = max(0.0, _mean2(dJZ_L, dJZ_R) * dL + rod)

            total_mass += M

            # Emit body block
            f.write(f"# node {i}: x=[{_f(xL)}, {_f(xR)}], dL={_f(dL)}\n")
            f.write(f"body: CURR_ROTOR + CURR_{name} + {i}, CURR_ROTOR + CURR_{name} + {i}\n")
            f.write( "    ,\n")
            f.write(f"    {_fe(M)},\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + BODY + {i}, 0., 0., 0.,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + BODY + {i},\n")
            f.write(f"        diag, {_fe(JX)}, {_fe(JY)}, {_fe(JZ)}\n")
            f.write( ";\n\n")

        f.write(f"# total_mass = {_fe(total_mass)}\n")
