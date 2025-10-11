# aero.py
"""
Aerodynamic references and beam panels.

blade_aero.ref:
- For each grid node i, define AERO+i co-located and co-oriented with FEATH+i.

blade.aerobeam:
- For each element, define 'aerodynamic beam3:' with 3 references (end-mid-end).
- Chord defined as piecewise linear over [-1,1] with three points (x1, xm, x2).
- BC point = -0.5 * Chord.
- Twist = const, 0.  (FEATH references carry structural pitch; if you need to pass
  aerodynamic twist, later change this to pull from AeroData.Twist)
"""

import math
from typing import Optional
from grid import BladeGrid
from io_blade import AeroData

# number formatting
def _f(val: float) -> str:
    return f"{val:.10f}"

def _fv(val: float) -> float:
    """finite value guard"""
    return float(val) if math.isfinite(val) else 0.0


def write_aero_refs_file(grid: BladeGrid, name: str, out_path: str) -> None:
    """
    Write blade_aero.ref: AERO+i references.

    File format (line style)
    ------------------------
    reference: CURR_ROTOR + CURR_{name} + AERO + {i}, #gen ref
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, 0., 0., 0.,
        reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, eye,
        reference, CURR_ROTOR + BASE, null,
        reference, CURR_ROTOR + BASE, null;
    """
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade_aero.ref\n")
        f.write(f"# name={name}\n")
        for i, _x in enumerate(grid.nodes, start=1):
            f.write(f"reference: CURR_ROTOR + CURR_{name} + AERO + {i}, #gen ref\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, 0., 0., 0.,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + FEATH + {i}, eye,\n")
            f.write(f"    reference, CURR_ROTOR + BASE, null,\n")
            f.write(f"    reference, CURR_ROTOR + BASE, null;\n\n")


def write_aero_beam_file(aero: Optional[AeroData],
                         grid: BladeGrid,
                         name: str,
                         out_path: str) -> None:
    """
    Write blade.aerobeam. If 'aero' is None, do nothing.

    For each element [x1,x2] with mid xm:
      - label = CURR_ROTOR + CURR_{name} + {elem_id}
      - references: AERO+(2i-1), AERO+(2i), AERO+(2i+1)
      - Piecewise linear chord with 3 points on xi ∈ {-1,0,+1}:
            value = Chord(x) via grid.interp_aero
      - AC = const, 0.
      - BC = piecewise linear with 3 points: value = -0.5 * Chord(x)
      - Twist = const, 0.
    """
    if aero is None:
        return
    if grid.interp_aero is None:
        raise RuntimeError("grid.interp_aero is not attached. Call attach_interpolators() after building grid and loading aero.")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# blade.aerobeam\n")
        f.write(f"# name={name}\n")
        for eidx, (x1, xm, x2) in enumerate(grid.elements, start=1):
            # structural node ids aligned with beam3 convention (end-mid-end)
            n1 = 2*eidx - 1
            n2 = 2*eidx
            n3 = 2*eidx + 1

            # chord values at end/mid/end (finite guard)
            c1 = _fv(grid.interp_aero("Chord", x1))
            cm = _fv(grid.interp_aero("Chord", xm))
            c2 = _fv(grid.interp_aero("Chord", x2))

            f.write(f"# element {eidx}: x1={_f(x1)} xm={_f(xm)} x2={_f(x2)}\n")
            f.write( "aerodynamic beam3:\n")
            f.write(f"    CURR_ROTOR + CURR_{name} + {eidx},   # aero panel name\n")
            f.write(f"    CURR_ROTOR + CURR_{name} + {eidx},   # link to structural beam\n")
            f.write( "    induced velocity, CURR_ROTOR,\n")
            # three references: end, mid, end
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + AERO + {n1}, null,\n")
            f.write( "        1, 0., 1., 0., 3, 1., 0., 0.,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + AERO + {n2}, null,\n")
            f.write( "        1, 0., 1., 0., 3, 1., 0., 0.,\n")
            f.write(f"    reference, CURR_ROTOR + CURR_{name} + AERO + {n3}, null,\n")
            f.write( "        1, 0., 1., 0., 3, 1., 0., 0.,\n")

            # Chord piecewise linear over xi = -1, 0, +1
            f.write( "    piecewise linear, 3,\n")
            f.write(f"        -1.0000000000, {_f(c1)},\n")
            f.write(f"         0.0000000000, {_f(cm)},\n")
            f.write(f"         1.0000000000, {_f(c2)},\n")

            # AC (aerodynamic center) const 0
            f.write( "    const, 0.,     # AC\n")

            # BC = -0.5 * Chord
            f.write( "    piecewise linear, 3,\n")
            f.write(f"        -1.0000000000, {_f(-0.5 * c1)},\n")
            f.write(f"         0.0000000000, {_f(-0.5 * cm)},\n")
            f.write(f"         1.0000000000, {_f(-0.5 * c2)},\n")

            # Twist const 0 (FEATH carries pitch)
            f.write( "    const, 0.,     # Twist\n\n")
