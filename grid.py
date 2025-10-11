# grid.py
"""
Grid construction and interpolation utilities for beam3 with end-mid-end nodes.

Given K control sections r0..rK-1:
- Nodes = [r0, mid01, r1, mid12, r2, ..., rK-1]  (length 2K-1)
- Elements = consecutive triplets (x1, xm, x2) over each control interval
- Evaluation points = xm ± 0.5/sqrt(3) * (x2 - x1)  (Gauss 2-point)

Provides interpolation closures for TipData (struct fields) and AeroData (Chord/Twist...).
"""

from dataclasses import dataclass
from typing import List, Tuple, Callable, Optional, Dict
import math
import numpy as np

from io_blade import TipData, AeroData


@dataclass
class BladeGrid:
    sections: List[float]                 # K control sections (strictly increasing)
    nodes: List[float]                    # 2K-1 end–mid–end nodes
    elements: List[Tuple[float, float, float]]   # [(x1,xm,x2)] length K-1
    eval_points: List[Tuple[float, float]]       # [(x_ev1,x_ev2)] length K-1
    # Interpolation callables are set by attach_interpolators()
    interp_tip: Optional[Callable[[str, float], float]] = None
    interp_aero: Optional[Callable[[str, float], float]] = None


def _strictly_increasing(xs: List[float]) -> bool:
    return all(xs[i+1] - xs[i] > 0.0 for i in range(len(xs)-1))


def build_grid(sections: List[float]) -> BladeGrid:
    """
    Build nodes/elements/eval_points from control sections.

    Parameters
    ----------
    sections : List[float]
        Sorted control stations (length K >= 2).

    Returns
    -------
    BladeGrid
        Constructed grid without interpolators.

    Raises
    ------
    ValueError
        If K < 2, or sections are not strictly increasing (duplicates/descending).
    """
    if sections is None or len(sections) < 2:
        raise ValueError("Need at least 2 control sections.")
    secs = [float(s) for s in sections]
    if not _strictly_increasing(secs):
        raise ValueError("Sections must be strictly increasing without duplicates.")

    # Build nodes: end–mid–end pattern for each interval
    nodes: List[float] = []
    elements: List[Tuple[float, float, float]] = []
    eval_points: List[Tuple[float, float]] = []

    inv_sqrt3 = 1.0 / math.sqrt(3.0)

    for i in range(len(secs) - 1):
        x1, x2 = secs[i], secs[i+1]
        xm = 0.5 * (x1 + x2)

        if i == 0:
            nodes.append(x1)   # first interval includes its left end

        nodes.append(xm)
        nodes.append(x2)

        elements.append((x1, xm, x2))

        half = 0.5 * (x2 - x1)
        x_ev1 = xm - 0.5 * inv_sqrt3 * (x2 - x1)
        x_ev2 = xm + 0.5 * inv_sqrt3 * (x2 - x1)
        eval_points.append((x_ev1, x_ev2))

    return BladeGrid(
        sections=secs,
        nodes=nodes,
        elements=elements,
        eval_points=eval_points,
        interp_tip=None,
        interp_aero=None,
    )


def _make_interp_closure(x: np.ndarray, field_map: Dict[str, np.ndarray]) -> Callable[[str, float], float]:
    """
    Build a closure: interp(field_name, x_query) -> float
    - Linear interpolation with clamping to [x_min, x_max]
    - field_name must exist in field_map; otherwise KeyError
    """
    x = np.asarray(x, dtype=float)
    if x.ndim != 1 or x.size == 0:
        raise ValueError("Interpolation abscissa must be a non-empty 1D array.")
    # ensure ascending; io.load_* should already give sorted, but double-guard here
    order = np.argsort(x)
    x = x[order]

    # reindex all fields to ascending order
    fmap: Dict[str, np.ndarray] = {}
    for k, v in field_map.items():
        arr = np.asarray(v, dtype=float)
        if arr.shape != x.shape:
            raise ValueError(f"Field '{k}' length {arr.shape} does not match x length {x.shape}.")
        fmap[k] = arr[order]

    x_min, x_max = float(x[0]), float(x[-1])

    def interp(field_name: str, xq: float) -> float:
        if field_name not in fmap:
            raise KeyError(f"Unknown field '{field_name}'.")
        y = fmap[field_name]
        # clamp then interp
        if xq <= x_min:
            return float(y[0])
        if xq >= x_max:
            return float(y[-1])
        return float(np.interp(xq, x, y))

    return interp


def attach_interpolators(grid: BladeGrid,
                         tip: TipData,
                         aero: Optional[AeroData]) -> BladeGrid:
    """
    Attach linear interpolation closures to grid:
      grid.interp_tip(field_name, x) -> float
      grid.interp_aero(field_name, x) -> float  (None if aero is None)

    Interpolation rules
    -------------------
    - Linear interpolation over the raw station arrays (STA or Radial).
    - Clamp outside to edge values.
    - Supported tip fields:
        ['EA','EJY','EJZ','GJ','YNA','ZNA','YCT','ZCT','ROTAN_deg','ROTAPI_deg',
         'dM','dJX','dJY','dJZ','YCG','ZCG']
    - Supported aero fields: at least ['Chord'] (others passthrough if provided).
    """
    # TipData fields map
    tip_field_map = {
        "EA":        np.asarray(tip.EA, dtype=float),
        "EJY":       np.asarray(tip.EJY, dtype=float),
        "EJZ":       np.asarray(tip.EJZ, dtype=float),
        "GJ":        np.asarray(tip.GJ, dtype=float),
        "YNA":       np.asarray(tip.YNA, dtype=float),
        "ZNA":       np.asarray(tip.ZNA, dtype=float),
        "YCT":       np.asarray(tip.YCT, dtype=float),
        "ZCT":       np.asarray(tip.ZCT, dtype=float),
        "ROTAN_deg": np.asarray(tip.ROTAN_deg, dtype=float),
        "ROTAPI_deg":np.asarray(tip.ROTAPI_deg, dtype=float),
        "dM":        np.asarray(tip.dM, dtype=float),
        "dJX":       np.asarray(tip.dJX, dtype=float),
        "dJY":       np.asarray(tip.dJY, dtype=float),
        "dJZ":       np.asarray(tip.dJZ, dtype=float),
        "YCG":       np.asarray(tip.YCG, dtype=float),
        "ZCG":       np.asarray(tip.ZCG, dtype=float),
    }
    grid.interp_tip = _make_interp_closure(np.asarray(tip.STA, dtype=float), tip_field_map)

    # AeroData fields map (optional)
    if aero is not None:
        aero_field_map = {
            "Chord": np.asarray(aero.Chord, dtype=float),
        }
        # Optional passthroughs if present (lengths already aligned in io.load_aero)
        if aero.Twist is not None:
            aero_field_map["Twist"] = np.asarray(aero.Twist, dtype=float)
        if aero.Sweep is not None:
            aero_field_map["Sweep"] = np.asarray(aero.Sweep, dtype=float)
        if aero.Anhedral is not None:
            aero_field_map["Anhedral"] = np.asarray(aero.Anhedral, dtype=float)

        grid.interp_aero = _make_interp_closure(np.asarray(aero.Radial, dtype=float), aero_field_map)
    else:
        grid.interp_aero = None

    return grid
