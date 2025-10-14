import math
from io_blade import TipData
from grid import BladeGrid


def _fe(x: float) -> str:
    return f"{x:.6e}"


def _f(x: float) -> str:
    return f"{x:.10f}"


def _assemble_K(EA, EJY, EJZ, GJ, Y1, Z1, rot_deg, nu):
    c = math.cos(math.radians(rot_deg)); s = math.sin(math.radians(rot_deg))
    GA = EA / (2 * (1 + nu))
    A22 = EJY * c * c + EJZ * s * s + (Z1 ** 2) * EA
    A33 = EJZ * c * c + EJY * s * s + (Y1 ** 2) * EA
    A23 = (EJY - EJZ) * c * s - Y1 * Z1 * EA
    return [
        [EA, 0, 0, 0, Z1 * EA, -Y1 * EA],
        [0, GA, 0, 0, 0, 0],
        [0, 0, GA, 0, 0, 0],
        [0, 0, 0, GJ, 0, 0],
        [Z1 * EA, 0, 0, 0, A22, A23],
        [-Y1 * EA, 0, 0, 0, A23, A33],
    ]


def write_beam_file(tip: TipData, grid: BladeGrid, name: str, nu: float, out_path: str, y_sign: float = 1.0) -> None:
    if grid.interp_tip is None:
        raise RuntimeError("attach_interpolators() first.")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.beam\n")
        f.write(f"# name={name}\n")
        for eidx, (x1, xm, x2) in enumerate(grid.elements, start=1):
            n1, n2, n3 = 2 * eidx - 1, 2 * eidx, 2 * eidx + 1

            def K_at(xev: float):
                EA = grid.interp_tip("EA", xev)
                EJY = grid.interp_tip("EJY", xev)
                EJZ = grid.interp_tip("EJZ", xev)
                GJ = grid.interp_tip("GJ", xev)
                rot = grid.interp_tip("ROTAN_deg", xev) * y_sign
                yct = y_sign * grid.interp_tip("YCT", xev)
                zct = grid.interp_tip("ZCT", xev)
                yna = y_sign * grid.interp_tip("YNA", xev)
                zna = grid.interp_tip("ZNA", xev)
                Y1 = yct - yna
                Z1 = zct - zna
                return _assemble_K(EA, EJY, EJZ, GJ, Y1, Z1, rot, nu)

            K1 = K_at(xm - 0.5 / math.sqrt(3) * (x2 - x1))
            K2 = K_at(xm + 0.5 / math.sqrt(3) * (x2 - x1))

            f.write(f"# element {eidx}: x1={_f(x1)} xm={_f(xm)} x2={_f(x2)}\n")
            f.write("beam3:\n")
            f.write(f"    CURR_ROTOR + CURR_blade + {eidx},\n")
            for nid in (n1, n2, n3):
                f.write(f"    CURR_ROTOR + CURR_blade + {nid}\n")
                f.write("        position, reference, CURR_ROTOR + CURR_blade + NEUTR + ")
                f.write(f"{nid}, 0., ")
                yct = y_sign * grid.interp_tip("YCT", grid.nodes[nid - 1])
                zct = grid.interp_tip("ZCT", grid.nodes[nid - 1])
                yna = y_sign * grid.interp_tip("YNA", grid.nodes[nid - 1])
                zna = grid.interp_tip("ZNA", grid.nodes[nid - 1])
                f.write(f"{_f(yct - yna)}, {_f(zct - zna)},\n")
                f.write(f"        orientation, reference, CURR_ROTOR + CURR_blade + NEUTR + {nid}, eye,\n")
            f.write("        from nodes,\n")
            for sec, K in enumerate((K1, K2), start=1):
                f.write("        linear elastic generic, matr,\n")
                for r in range(6):
                    row = ", ".join(_fe(K[r][c]) for c in range(6))
                    end = ";\n" if r == 5 else ",\n"
                    f.write(f"            {row}{end}")
