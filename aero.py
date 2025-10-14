from typing import Optional


from grid import BladeGrid
from io_blade import AeroData


def write_aero_refs_file(grid: BladeGrid, name: str, out_path: str) -> None:
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade_aero.ref\n")
        f.write(f"# name={name}\n")
        for i, _x in enumerate(grid.nodes, start=1):
            f.write(f"reference: CURR_ROTOR + CURR_blade + AERO + {i}, #gen ref\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, 0., 0., 0.,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_blade + FEATH + {i}, eye,\n")
            f.write("    reference, CURR_ROTOR + BASE, null,\n")
            f.write("    reference, CURR_ROTOR + BASE, null;\n\n")


def write_aero_beam_file(aero: Optional[AeroData], grid: BladeGrid, name: str, out_path: str) -> None:
    if aero is None:
        return
    if grid.interp_aero is None:
        raise RuntimeError("attach_interpolators() first.")

    def _f(x: float) -> str:
        return f"{x:.10f}"

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.aerobeam\n")
        f.write(f"# name={name}\n")
        for eidx, (x1, xm, x2) in enumerate(grid.elements, start=1):
            n1, n2, n3 = 2 * eidx - 1, 2 * eidx, 2 * eidx + 1
            c1 = grid.interp_aero("Chord", x1)
            cm = grid.interp_aero("Chord", xm)
            c2 = grid.interp_aero("Chord", x2)
            f.write(f"# element {eidx}: x1={_f(x1)} xm={_f(xm)} x2={_f(x2)}\n")
            f.write("aerodynamic beam3:\n")
            f.write(f"    CURR_ROTOR + CURR_blade + {eidx},\n")
            f.write(f"    CURR_ROTOR + CURR_blade + {eidx},\n")
            f.write("    induced velocity, CURR_ROTOR,\n")
            for nid in (n1, n2, n3):
                f.write(f"    reference, CURR_ROTOR + CURR_blade + AERO + {nid}, null,\n")
                f.write("        1, 0., 1., 0., 3, 1., 0., 0.,\n")
            f.write("    piecewise linear, 3,\n")
            f.write(f"        -1.0000000000, {_f(c1)},\n")
            f.write(f"         0.0000000000, {_f(cm)},\n")
            f.write(f"         1.0000000000, {_f(c2)},\n")
            f.write("    const, 0.,\n")
            f.write("    piecewise linear, 3,\n")
            f.write(f"        -1.0000000000, {_f(-0.5 * c1)},\n")
            f.write(f"         0.0000000000, {_f(-0.5 * cm)},\n")
            f.write(f"         1.0000000000, {_f(-0.5 * c2)},\n")
            f.write("    const, 0.,\n\n")
