# select_sections.py
"""
Automatic control section selection to replace manual `section_blade = ...`.

Goals:
- Minimize user input and errors
- Capture geometry/stiffness changes (jumps, chord vertices)
- Control fidelity via error-based refinement
- Bound element sizes and total count
- Provide an interpretable report

Definitions:
- K control sections -> (K-1) elements -> (2K-1) nodes (end-mid-end pattern)
"""

from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
import math
import numpy as np

from io_blade import TipData, AeroData


@dataclass
class SectionSelectionConfig:
    r_start: Optional[float] = None  # if None, auto detect
    err_tol: float = 0.05            # midpoint relative error tolerance
    jump_tol: float = 0.10           # relative jump between adjacent stations
    max_elems: int = 40              # hard cap on elements
    max_dr: Optional[float] = None   # max segment length
    min_dr: Optional[float] = None   # min segment length
    c_eps: float = 1e-3              # chord > c_eps to auto detect blade start


@dataclass
class SectionReport:
    sections: List[float]
    reasons: Dict[float, List[str]]  # map section r -> reasons ['START','END','JUMP:EA','ERR>tol',...]
    r_start_used: float
    elems: int
    nodes: int
    warnings: List[str]
    notes: List[str]


# ───────────────────────── helpers ─────────────────────────

_EPS = 1e-12

def _interp1(x: np.ndarray, y: np.ndarray, xi: float) -> float:
    """Linear interpolation with clamping to [min(x), max(x)]."""
    if len(x) == 0:
        return 0.0
    if xi <= x[0]:
        return float(y[0])
    if xi >= x[-1]:
        return float(y[-1])
    return float(np.interp(xi, x, y))

def _add_reason(reasons: Dict[float, List[str]], r: float, why: str):
    reasons.setdefault(r, [])
    if why not in reasons[r]:
        reasons[r].append(why)

def _is_protected(reasons: Dict[float, List[str]], r: float) -> bool:
    """Hard constraints we try not to merge away."""
    tags = reasons.get(r, [])
    for t in tags:
        if t.startswith("START") or t.startswith("END") or t.startswith("JUMP") or t.startswith("VERTEX"):
            return True
    return False

def _auto_r_start_from_aero(aero: Optional[AeroData], c_eps: float, rmin: float, rmax: float) -> Optional[float]:
    if aero is None or not aero.Chord:
        return None
    r = np.asarray(aero.Radial, dtype=float)
    c = np.asarray(aero.Chord, dtype=float)
    if np.all(~np.isfinite(c)) or np.all(np.abs(c) <= 0):
        return None
    idx = np.where(c > c_eps)[0]
    if len(idx) == 0:
        return None
    r0 = float(r[idx[0]])
    # clamp to [rmin, rmax)
    return max(rmin, min(r0, max(rmin, rmax - _EPS)))

def _auto_r_start_from_struct(tip: TipData) -> float:
    r = np.asarray(tip.STA, dtype=float)
    EA  = np.asarray(tip.EA, dtype=float)
    EJY = np.asarray(tip.EJY, dtype=float)
    EJZ = np.asarray(tip.EJZ, dtype=float)
    GJ  = np.asarray(tip.GJ, dtype=float)
    # Use relative threshold vs global max to avoid noise at root
    mags = np.vstack([np.abs(EA), np.abs(EJY), np.abs(EJZ), np.abs(GJ)])
    maxv = np.max(mags)
    thr = 1e-6 * max(1.0, maxv)  # permissive
    for i in range(len(r)):
        if EA[i] > thr or EJY[i] > thr or EJZ[i] > thr or GJ[i] > thr:
            return float(r[i])
    return float(r[0])

def _detect_jumps(x: np.ndarray, y: np.ndarray, r_start: float, jump_tol: float) -> List[float]:
    """Return x positions (from original samples) where relative jump exceeds threshold."""
    rs = []
    for i in range(1, len(x)):
        if x[i] < r_start:
            continue
        y0, y1 = y[i-1], y[i]
        base = max(abs(y0), abs(y1), _EPS)
        rel = abs(y1 - y0) / base
        if rel >= jump_tol:
            rs.append(float(x[i]))
    return rs

def _detect_chord_vertices(aero: Optional[AeroData], r_start: float) -> List[float]:
    """
    Detect local extrema of chord to capture planform vertices.
    Simple criterion: (c[i]-c[i-1])*(c[i+1]-c[i]) <= 0 and
      max(|ΔL|,|ΔR|) > δ, where δ = 1e-3 * max(chord).
    """
    if aero is None or not aero.Chord or len(aero.Radial) < 3:
        return []
    r = np.asarray(aero.Radial, dtype=float)
    c = np.asarray(aero.Chord, dtype=float)
    if np.max(np.abs(c)) <= 0:
        return []
    delta_thr = 1e-3 * float(np.max(np.abs(c)))
    out = []
    for i in range(1, len(r)-1):
        if r[i] < r_start:
            continue
        dL = c[i] - c[i-1]
        dR = c[i+1] - c[i]
        if dL * dR <= 0 and max(abs(dL), abs(dR)) > delta_thr:
            out.append(float(r[i]))
    return out

def _mid_error(x: np.ndarray, y: np.ndarray, a: float, b: float) -> float:
    """Relative midpoint error of linear interpolation vs true value."""
    if b <= a + _EPS:
        return 0.0
    rm = 0.5*(a+b)
    ya = _interp1(x, y, a)
    yb = _interp1(x, y, b)
    ym = _interp1(x, y, rm)
    ylin = ya + (yb - ya) * (rm - a) / (b - a)
    denom = max(abs(ym), abs(ya), abs(yb), _EPS)
    return abs(ym - ylin) / denom

def _ensure_sorted_unique(vals: List[float]) -> List[float]:
    arr = np.array(sorted(vals), dtype=float)
    if len(arr) == 0:
        return []
    uniq = [arr[0]]
    for v in arr[1:]:
        if v - uniq[-1] > 1e-12:
            uniq.append(v)
    return uniq

# ───────────────────────── main routine ─────────────────────────

def auto_select_sections(tip: TipData,
                         aero: Optional[AeroData],
                         cfg: SectionSelectionConfig) -> Tuple[List[float], SectionReport]:
    """
    Automatic section selection.

    Algorithm (summary)
    -------------------
    1) Determine r_start:
       - If aero provided: first Radial where Chord > c_eps.
       - Else: first STA where any of {EA, EJY, EJZ, GJ} is non-zero-ish.
       - Allow cfg.r_start to override, if provided (clamped to domain).

    2) Hard constraints:
       - Include START (r_start) and END (max STA).
       - Include "jump" points where |f[i+1]-f[i]| > jump_tol * max(|f[i]|,|f[i+1]|, eps)
         for f in {Chord (if aero), EA, EJY, EJZ, GJ}.
       - Include chord vertices (slope change) if aero available.

    3) Error-based refinement (loop until done or max_elems):
       For each segment [ri, rj], compute midpoint error for signals in
       S = {Chord (if aero), EA, EJY, EJZ, GJ}.
       If any eps_f > err_tol, insert rm as new control section.

    4) Size constraints:
       - Enforce max_dr by subdividing too-long segments (equal partition).
       - Enforce min_dr by merging tiny segments if possible (retain hard constraints).

    5) Deduplicate and sort sections; compute sizes; report.
    """
    # Domain from structural data
    r_tip = np.asarray(tip.STA, dtype=float)
    rmin, rmax = float(r_tip[0]), float(r_tip[-1])

    # 1) r_start
    if cfg.r_start is not None:
        r0 = float(cfg.r_start)
    else:
        r0 = _auto_r_start_from_aero(aero, cfg.c_eps, rmin, rmax) or _auto_r_start_from_struct(tip)
    # clamp
    if r0 < rmin: r0 = rmin
    if r0 >= rmax: r0 = max(rmin, rmax - _EPS)

    # 2) Hard constraints
    sections: List[float] = [r0, rmax]
    reasons: Dict[float, List[str]] = {}
    _add_reason(reasons, r0, "START")
    _add_reason(reasons, rmax, "END")

    # Signals on native grids
    # Structural
    EA  = np.asarray(tip.EA, dtype=float)
    EJY = np.asarray(tip.EJY, dtype=float)
    EJZ = np.asarray(tip.EJZ, dtype=float)
    GJ  = np.asarray(tip.GJ, dtype=float)

    # Jumps on structural stations
    for name, vec in (("EA", EA), ("EJY", EJY), ("EJZ", EJZ), ("GJ", GJ)):
        js = _detect_jumps(r_tip, vec, r0, cfg.jump_tol)
        for rr in js:
            if rr >= r0 and rr <= rmax:
                sections.append(rr)
                _add_reason(reasons, rr, f"JUMP:{name}")

    # Chord jumps & vertices (if aero)
    if aero is not None and aero.Chord:
        r_a = np.asarray(aero.Radial, dtype=float)
        c_a = np.asarray(aero.Chord, dtype=float)
        js_c = _detect_jumps(r_a, c_a, r0, cfg.jump_tol)
        vs_c = _detect_chord_vertices(aero, r0)
        for rr in js_c:
            if rr >= r0 and rr <= rmax:
                sections.append(rr)
                _add_reason(reasons, rr, "JUMP:Chord")
        for rr in vs_c:
            if rr >= r0 and rr <= rmax:
                sections.append(rr)
                _add_reason(reasons, rr, "VERTEX:Chord")

    # Deduplicate base set
    sections = _ensure_sorted_unique(sections)

    # 4a) Enforce max_dr first (equal partitions)
    if cfg.max_dr is not None and cfg.max_dr > 0:
        changed = True
        while changed:
            changed = False
            new_pts: List[float] = []
            for a, b in zip(sections[:-1], sections[1:]):
                dr = b - a
                if dr > cfg.max_dr + _EPS:
                    n = int(math.ceil(dr / cfg.max_dr))
                    for k in range(1, n):
                        rp = a + dr * k / n
                        new_pts.append(rp)
            if new_pts:
                for rp in new_pts:
                    if rp not in reasons:
                        _add_reason(reasons, rp, "MAX_DR")
                sections = _ensure_sorted_unique(sections + new_pts)
                changed = True
            if len(sections) - 1 >= cfg.max_elems:
                break

    # 3) Error-based refinement
    def err_any(a: float, b: float) -> Tuple[float, float]:
        """Return (max_err, err_from_which_signal_as_code)."""
        errs = []
        # structural signals on tip grid
        for code, vec in (("EA", EA), ("EJY", EJY), ("EJZ", EJZ), ("GJ", GJ)):
            e = _mid_error(r_tip, vec, a, b)
            errs.append((e, code))
        # chord if available
        if aero is not None and aero.Chord:
            r_a = np.asarray(aero.Radial, dtype=float)
            c_a = np.asarray(aero.Chord, dtype=float)
            errs.append((_mid_error(r_a, c_a, a, b), "Chord"))
        # choose max
        emax, code = max(errs, key=lambda t: t[0]) if errs else (0.0, "NA")
        return emax, code

    changed = True
    warnings: List[str] = []
    while changed and (len(sections) - 1) < cfg.max_elems:
        changed = False
        new_pts: List[Tuple[float, str]] = []
        for a, b in zip(sections[:-1], sections[1:]):
            dr = b - a
            # Respect min_dr: if already very small, don't split further
            if cfg.min_dr is not None and dr <= cfg.min_dr + _EPS:
                continue
            emax, code = err_any(a, b)
            if emax > cfg.err_tol:
                rm = 0.5*(a+b)
                new_pts.append((rm, f"ERR>{cfg.err_tol:.3f}:{code}"))
        if new_pts:
            for rp, why in new_pts:
                _add_reason(reasons, rp, why)
            sections = _ensure_sorted_unique(sections + [rp for rp, _ in new_pts])
            changed = True
        if len(sections) - 1 >= cfg.max_elems:
            break

    # 4b) Enforce min_dr by merging removable interior points
    if cfg.min_dr is not None and cfg.min_dr > 0:
        changed = True
        while changed:
            changed = False
            if len(sections) <= 2:
                break
            # compute drs
            drs = [b - a for a, b in zip(sections[:-1], sections[1:])]
            # if any very small segments exist, attempt merge by removing a non-protected interior point
            for i in range(1, len(sections)-1):
                left_dr = sections[i]   - sections[i-1]
                right_dr= sections[i+1] - sections[i]
                if min(left_dr, right_dr) < cfg.min_dr - _EPS and not _is_protected(reasons, sections[i]):
                    # remove this interior point
                    warnings.append(f"min_dr merge: removed section at r={sections[i]:.6f}")
                    reasons.pop(sections[i], None)
                    sections.pop(i)
                    changed = True
                    break  # restart scan

    # Final dedupe & cap max_elems if necessary (drop newest interior points without hard tags)
    sections = _ensure_sorted_unique(sections)
    while (len(sections) - 1) > cfg.max_elems:
        # try to drop a non-protected interior section (closest to mid-span)
        mid = 0.5*(sections[0] + sections[-1])
        candidates = [(abs(r - mid), idx, r) for idx, r in enumerate(sections[1:-1], start=1) if not _is_protected(reasons, r)]
        if not candidates:
            warnings.append("max_elems limit reached but cannot drop protected sections.")
            break
        _, idx_drop, r_drop = sorted(candidates)[0]
        reasons.pop(r_drop, None)
        sections.pop(idx_drop)

    sections = _ensure_sorted_unique(sections)
    K = len(sections)
    elems = max(0, K - 1)
    nodes = max(0, 2*K - 1)

    report = SectionReport(
        sections=sections,
        reasons=reasons,
        r_start_used=r0,
        elems=elems,
        nodes=nodes,
        warnings=list(dict.fromkeys(warnings)),  # dedupe keep order
        notes=[
            f"signals={'Chord+' if (aero and aero.Chord) else ''}EA,EJY,EJZ,GJ",
            f"err_tol={cfg.err_tol}, jump_tol={cfg.jump_tol}, max_elems={cfg.max_elems}, "
            f"max_dr={cfg.max_dr}, min_dr={cfg.min_dr}"
        ]
    )
    return sections, report
