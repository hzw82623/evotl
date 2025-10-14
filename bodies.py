from io_blade import TipData
from grid import BladeGrid


def _fe(x: float) -> str:
    return f"{x:.6e}"


def _f(x: float) -> str:
    return f"{x:.10f}"


def _mean2(a, b):
    return 0.5 * (a + b)


def write_bodies_file(tip: TipData, grid: BladeGrid, name: str, out_path: str) -> None:
    if grid.interp_tip is None:
        raise RuntimeError("attach_interpolators() first.")
    nodes = grid.nodes
    N = len(nodes)
    sections = grid.sections
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.body\n")
        f.write(f"# name={name}\n")
        total = 0.0
        for i, _x in enumerate(nodes, start=1):
            if i == 1:
                xL = sections[0]
                xR = 0.5 * (nodes[0] + nodes[1]) if N > 1 else sections[-1]
            elif i == N:
                xL = 0.5 * (nodes[N - 2] + nodes[N - 1]) if N > 1 else sections[0]
                xR = sections[-1]
            else:
                xL = 0.5 * (nodes[i - 2] + nodes[i - 1])
                xR = 0.5 * (nodes[i - 1] + nodes[i])
            dL = max(0.0, xR - xL)
            if dL <= 0:
                M = JX = JY = JZ = 0.0
            else:
                dM_L = grid.interp_tip("dM", xL)
                dM_R = grid.interp_tip("dM", xR)
                dJX_L = grid.interp_tip("dJX", xL)
                dJX_R = grid.interp_tip("dJX", xR)
                dJY_L = grid.interp_tip("dJY", xL)
                dJY_R = grid.interp_tip("dJY", xR)
                dJZ_L = grid.interp_tip("dJZ", xL)
                dJZ_R = grid.interp_tip("dJZ", xR)
                M = max(0.0, _mean2(dM_L, dM_R) * dL)
                JX = max(0.0, _mean2(dJX_L, dJX_R) * dL)
                rod = (M * dL * dL) / 12.0
                JY = max(0.0, _mean2(dJY_L, dJY_R) * dL + rod)
                JZ = max(0.0, _mean2(dJZ_L, dJZ_R) * dL + rod)
            total += M
            f.write(f"# node {i}: x=[{_f(xL)}, {_f(xR)}], dL={_f(dL)}\n")
            f.write(f"body: CURR_ROTOR + CURR_blade + {i}, CURR_ROTOR + CURR_blade + {i}\n")
            f.write("    ,\n")
            f.write(f"    {_fe(M)},\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + BODY + {i}, 0., 0., 0.,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + BODY + {i},\n")
            f.write(f"        diag, {_fe(JX)}, {_fe(JY)}, {_fe(JZ)}\n")
            f.write(";\n\n")
        f.write(f"# total_mass = {_fe(total)}\n")
