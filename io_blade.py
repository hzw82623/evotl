# io.py
"""
I/O and preprocessing for blade-only pipeline.

- Read XV-15 style .tip:
  Header includes at least: STA WEIGHT XCG ZCG ROTAPI JX JZ JP EA XNA ZNA ROTAN EJZ EJX GJ XCT ZCT
  There may be a leading 'SEC' or row index column in data lines.
  A "units" line may follow header (containing tokens like 'lb', 'ft', 'DEG', 'ADIM', 'slug', '**' etc.).
  Data lines are whitespace (or tab) separated; scientific 'E' notation allowed.

- Read aero .dat:
  Header: Radial Chord Twist Sweep Anhedral (Twist/Sweep/Anhedral optional)
  Next line often is units; rest are numeric rows.

Coordinate mapping (performed here, once):
  TIP axes X/Z are mapped to beam local y/z.
  After mapping, downstream modules ONLY see (y,z)-based names:
    EJY <- EJX,  EJZ <- EJZ
    YNA <- ZNA,  ZNA <- XNA
    YCT <- ZCT,  ZCT <- XCT
    YCG <- ZCG,  ZCG <- XCG
    dJY <- JZ,   dJZ <- JX,   dJX <- JP
"""

from dataclasses import dataclass
from typing import List, Optional, Dict, Any, Tuple
import math
import io as _stdlib_io  # avoid shadowing when needed
import os

# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class TipData:
    STA:   List[float]
    dM:    List[float]   # from WEIGHT (line mass density; units kept 'raw')
    YCG:   List[float]
    ZCG:   List[float]
    ROTAPI_deg: List[float]
    ROTAN_deg:  List[float]
    dJX:   List[float]   # from JP (polar) — kept as provided
    dJY:   List[float]   # from JZ (X/Z → y/z mapping)
    dJZ:   List[float]   # from JX (X/Z → y/z mapping)
    EA:    List[float]
    EJY:   List[float]   # from EJX (X/Z → y/z mapping)
    EJZ:   List[float]
    GJ:    List[float]
    YNA:   List[float]   # from ZNA
    ZNA:   List[float]   # from XNA
    YCT:   List[float]   # from ZCT
    ZCT:   List[float]   # from XCT
    meta:  Dict[str, Any]  # e.g., {'warnings': [...], 'header': [...], 'cols': {...}, 'units_policy': 'raw'}


@dataclass
class AeroData:
    Radial:   List[float]
    Chord:    List[float]
    Twist:    Optional[List[float]] = None
    Sweep:    Optional[List[float]] = None
    Anhedral: Optional[List[float]] = None
    meta:     Dict[str, Any] = None

# ─────────────────────────────────────────────────────────────────────────────
# Helpers

def _is_numeric_row(tokens: List[str]) -> bool:
    """Return True if all tokens parse as float."""
    if not tokens:
        return False
    try:
        for t in tokens:
            float(t)
        return True
    except Exception:
        return False

def _tokenize(line: str) -> List[str]:
    return line.replace(",", " ").split()

def _looks_like_units_line(tokens: List[str]) -> bool:
    """Heuristic: a non-numeric line right after header containing units-ish tokens."""
    text = " ".join(tokens).lower()
    keys = ["deg", "adim", "unit", "lb", "slug", "ft", "m", "kg", "**", "[]", "rad", "in"]
    return any(k in text for k in keys)

def _dedupe_and_sort(x: List[float], cols: Dict[str, List[float]]) -> Tuple[List[float], Dict[str, List[float]]]:
    """Sort by x ascending and average duplicates (stable mean)."""
    import numpy as np
    x_arr = np.asarray(x, dtype=float)
    order = np.argsort(x_arr)
    x_sorted = x_arr[order]
    cols_sorted = {k: np.asarray(v, dtype=float)[order] for k, v in cols.items()}
    # group by unique x
    uniq, inv = np.unique(x_sorted, return_inverse=True)
    out = {k: np.zeros_like(uniq, dtype=float) for k in cols_sorted}
    counts = {k: np.bincount(inv, minlength=len(uniq)) for k in ["_dummy"]}
    for k, arr in cols_sorted.items():
        out[k] = np.bincount(inv, weights=arr, minlength=len(uniq)) / np.maximum(counts["_dummy"], 1)
    return uniq.tolist(), {k: v.tolist() for k, v in out.items()}

# ─────────────────────────────────────────────────────────────────────────────
# TIP reader

def load_tip(path: str, units_policy: str = "si") -> TipData:
    """
    Robust TIP reader for XV-15 style files:
    - Locate BLADE STRUCT Y table blocks (TABLE ... ENDTABLE)
    - Prefer the block whose units line is metric (STA->m, WEIGHT->kg/m)
    - Map columns by NAME (case/units-insensitive, substring ok); ignore "..." drift
    - Return TipData with X/Z -> y/z mapping applied
    """
    import os, re
    if not os.path.isfile(path):
        raise FileNotFoundError(f"TIP file not found: {path}")

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        raw = [ln.rstrip("\n") for ln in f]

    def toks(s: str):
        return s.replace(",", " ").split()

    # 1) 找到所有 BLADE STRUCT Y 的 TABLE…ENDTABLE 区块
    blocks = []
    i = 0
    while i < len(raw):
        if "BLADE" in raw[i].upper() and "STRUCT" in raw[i].upper() and "Y" in raw[i].upper():
            # 向下寻找 TABLE
            j = i + 1
            while j < len(raw) and "TABLE" not in raw[j].upper():
                j += 1
            if j >= len(raw): break
            # 取 header + units + data 到 ENDTABLE
            hdr_line = j + 1
            units_line = j + 2 if j + 2 < len(raw) else None
            k = j + 3
            data_lines = []
            while k < len(raw) and "ENDTABLE" not in raw[k].upper():
                if raw[k].strip() and not raw[k].strip().startswith(("#","!","//")):
                    data_lines.append(raw[k])
                k += 1
            if data_lines:
                blocks.append((hdr_line, units_line, data_lines))
            i = k + 1
        else:
            i += 1

    if not blocks:
        raise ValueError("No STRUCT Y TABLE found in TIP.")

    # 2) 选择 SI 表（units 第二列是 m，第三列是 kg/m）
    def norm(s: str) -> str:
        s = s.upper().strip()
        s = re.sub(r"[^A-Z0-9]", "", s)  # 去非字母数字
        return s

    def parse_block(block):
        hdr_idx, units_idx, data_lines = block
        hdr = toks(raw[hdr_idx]) if hdr_idx is not None else []
        units = toks(raw[units_idx]) if units_idx is not None else []
        # 规范化表头：允许 "…ROTAN" 这种含省略号，做包含匹配
        want = ["SEC","STA","WEIGHT","XCG","ZCG","ROTAPI","JX","JZ","JP",
                "EA","XNA","ZNA","ROTAN","EJZ","EJX","GJ","XCT","ZCT"]
        idx_map = {}
        hdr_norm = [norm(h) for h in hdr]
        for w in want:
            for j, h in enumerate(hdr_norm):
                if w in h:  # 包含匹配
                    idx_map[w] = j
                    break
        # 解析数据行为二维数组
        rows = [toks(ln) for ln in data_lines if ln.strip()]
        rows = [[float(x) for x in r] for r in rows if all(_x.replace(".","",1).replace("-","",1).replace("e","1").replace("E","1").lstrip().replace("+","",1) or True for _x in r)]
        return idx_map, units, rows

    parsed = [parse_block(b) for b in blocks]

    # 简单地判断 SI：units 第二个 token 是否包含 'M'（STA 单位），第三个 token 是否包含 'KG/M'
    def is_si(units):
        u = [norm(x) for x in units]
        return len(u) >= 3 and ("M" == u[1] or "M" in u[1]) and ("KGM" == u[2] or "KGM" in u[2])
    # 选择最优块：优先 SI，否则第一个
    chosen = None
    for p in parsed:
        if is_si(p[1]):
            chosen = p
            break
    if chosen is None:
        chosen = parsed[0]

    idx_map, units_used, rows = chosen
    # 必要列检查（至少 STA & WEIGHT 要有）
    if "STA" not in idx_map or "WEIGHT" not in idx_map:
        # 尝试按 XV-15 SI 表的固定顺序兜底（SEC, STA, WEIGHT, XCG, ZCG, ROTAPI, JX, JZ, JP, EA, XNA, ZNA, ROTAN, EJZ, EJX, GJ, XCT, ZCT）
        if rows and all(len(r) >= 18 for r in rows):
            fixed = ["SEC","STA","WEIGHT","XCG","ZCG","ROTAPI","JX","JZ","JP",
                     "EA","XNA","ZNA","ROTAN","EJZ","EJX","GJ","XCT","ZCT"]
            idx_map = {name: i for i, name in enumerate(fixed)}
        else:
            raise ValueError("TIP header mapping failed: cannot locate STA/WEIGHT.")

    def col(name, default0=False):
        if name not in idx_map:
            if default0:
                return [0.0]*len(rows)
            raise KeyError(f"Missing column '{name}'")
        j = idx_map[name]
        return [ (r[j] if j < len(r) else 0.0) for r in rows ]

    # 3) 按列名取值（随后做 X/Z→y/z 映射）
    STA   = col("STA")
    WEI   = col("WEIGHT")
    XCG   = col("XCG", True); ZCG = col("ZCG", True)
    ROTAPI= col("ROTAPI", True)
    JX    = col("JX", True);  JZ  = col("JZ", True); JP = col("JP", True)
    EA    = col("EA", True)
    XNA   = col("XNA", True); ZNA = col("ZNA", True)
    ROTAN = col("ROTAN", True)
    EJZ   = col("EJZ", True); EJX = col("EJX", True)
    GJ    = col("GJ", True)
    XCT   = col("XCT", True); ZCT = col("ZCT", True)

    # 排序 & 去重（按 STA 升序）
    import numpy as np
    STA_arr = np.asarray(STA, dtype=float)
    order = np.argsort(STA_arr)
    def reord(v): 
        a = np.asarray(v, dtype=float)
        return a[order].tolist()
    STA   = STA_arr[order].tolist()
    WEI   = reord(WEI)
    XCG   = reord(XCG); ZCG = reord(ZCG)
    ROTAPI= reord(ROTAPI)
    JX    = reord(JX);  JZ  = reord(JZ); JP = reord(JP)
    EA    = reord(EA)
    XNA   = reord(XNA); ZNA = reord(ZNA)
    ROTAN = reord(ROTAN)
    EJZ   = reord(EJZ); EJX = reord(EJX)
    GJ    = reord(GJ)
    XCT   = reord(XCT); ZCT = reord(ZCT)

    # 4) X/Z → y/z 映射（一次性做完，后续模块只用 y/z 命名）
    tip = TipData(
        STA = STA,
        dM  = WEI,
        YCG = ZCG,         # from ZCG
        ZCG = XCG,         # from XCG
        ROTAPI_deg = ROTAPI,
        ROTAN_deg  = ROTAN,
        dJX = JP,          # from JP
        dJY = JZ,          # from JZ (X->y)
        dJZ = JX,          # from JX (X->z)
        EA  = EA,
        EJY = EJX,         # from EJX
        EJZ = EJZ,
        GJ  = GJ,
        YNA = ZNA,         # from ZNA
        ZNA = XNA,         # from XNA
        YCT = ZCT,         # from ZCT
        ZCT = XCT,         # from XCT
        meta = {"units_policy": units_policy, "units_row": units_used}
    )
    return tip

# ─────────────────────────────────────────────────────────────────────────────
# AERO reader

def load_aero(path: Optional[str], units_policy: str = "raw") -> Optional[AeroData]:
    """
    Parse aero .dat and return AeroData (Radial/Chord required; others optional).
    - Robust header parsing: accepts synonyms and tokens with units/parentheses.
    - Fallback heuristic if header mapping fails: choose most monotonic column as Radial,
      and the most varying other column as Chord.
    """
    if path is None:
        return None
    if not os.path.isfile(path):
        raise FileNotFoundError(f"AERO file not found: {path}")

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = [ln.strip() for ln in f if ln.strip() and not ln.strip().startswith(("#","!","//"))]

    header_tokens: Optional[List[str]] = None
    numeric_rows: List[List[float]] = []
    warnings: List[str] = []

    # ------- helpers (local) -------
    def _tokenize(s: str) -> List[str]:
        return s.replace(",", " ").split()

    def _is_numeric_row(tokens: List[str]) -> bool:
        try:
            for t in tokens: float(t)
            return True
        except: return False

    def _looks_like_units_line(tokens: List[str]) -> bool:
        text = " ".join(tokens).lower()
        for k in ["deg","adim","unit","lb","slug","ft","m","kg","**","[]","rad","in","mm","cm"]:
            if k in text: return True
        return False

    def _norm(tok: str) -> str:
        # uppercase + strip non-alnum, remove trailing units/parentheses etc.
        t = tok.upper().strip()
        t = "".join(ch for ch in t if ch.isalnum())
        return t

    RAD_KEYS = {"RADIAL","R","STA","RADIUS","RAD","STATION","SPAN"}
    CHD_KEYS = {"CHORD","C","CRD","CHRD","CH","CHORDM","CHORDMM","CHORDIN","CHORDLENGTH"}
    TWT_KEYS = {"TWIST","THETA","PITCH","TWISTDEG","TWISTANGLE"}
    SWP_KEYS = {"SWEEP"}
    ANH_KEYS = {"ANHEDRAL","DIHEDRAL","ANHD","ANH"}

    # ------- find header (first non-numeric), skip units lines -------
    i = 0
    while i < len(lines):
        toks = _tokenize(lines[i])
        if not _is_numeric_row(toks):
            header_tokens = toks
            j = i + 1
            while j < len(lines):
                t2 = _tokenize(lines[j])
                if _is_numeric_row(t2): break
                if _looks_like_units_line(t2): j += 1; continue
                header_tokens += t2
                j += 1
            i = j
            break
        else:
            break

    # data rows
    for k in range(i, len(lines)):
        toks = _tokenize(lines[k])
        if _is_numeric_row(toks):
            numeric_rows.append([float(t) for t in toks])

    if not numeric_rows:
        raise ValueError("No numeric data rows found in AERO.")

    # ------- map columns by header (robust) -------
    def _build_idx_map(hdr: List[str]) -> Dict[str,int]:
        idx: Dict[str,int] = {}
        for j, raw in enumerate(hdr):
            key = _norm(raw)
            if key in RAD_KEYS and "Radial" not in idx: idx["Radial"] = j
            elif key in CHD_KEYS and "Chord" not in idx: idx["Chord"]  = j
            elif key in TWT_KEYS and "Twist" not in idx: idx["Twist"]  = j
            elif key in SWP_KEYS and "Sweep" not in idx: idx["Sweep"]  = j
            elif key in ANH_KEYS and "Anhedral" not in idx: idx["Anhedral"] = j
        return idx

    idx_map: Dict[str,int] = {}
    if header_tokens is not None:
        idx_map = _build_idx_map(header_tokens)

    ncols = max(len(r) for r in numeric_rows)
    # materialize full columns with padding zeros for ragged rows
    cols = [[(row[j] if j < len(row) else 0.0) for row in numeric_rows] for j in range(ncols)]

    def _choose_by_heuristics() -> Tuple[int,int]:
        # pick the most (non-decreasing) monotonic as Radial; most varying other as Chord
        def mono_score(col: List[float]) -> float:
            if len(col) < 2: return 0.0
            nondec = sum(1 for k in range(1,len(col)) if col[k] >= col[k-1])
            return nondec / (len(col)-1)
        scores = [mono_score(c) for c in cols]
        rad_idx = int(max(range(ncols), key=lambda j: scores[j]))
        vari = [float(np.std(c)) if j != rad_idx else -1.0 for j, c in enumerate(cols)]
        # choose the remaining column with max std as chord
        cand = [j for j in range(ncols) if j != rad_idx]
        chord_idx = int(max(cand, key=lambda j: vari[j])) if cand else None
        return rad_idx, chord_idx

    import numpy as np

    if ("Radial" not in idx_map) or ("Chord" not in idx_map):
        warnings.append("AERO header lacks canonical names; trying heuristic column guess.")
        ridx, cidx = _choose_by_heuristics()
        if cidx is None:
            # as ultimate fallback, assume first two columns when available
            if ncols > 1:
                ridx, cidx = 0, 1
            else:
                ridx, cidx = 0, None
        if cidx is None:
            raise ValueError("AERO: could not infer Radial/Chord columns.")
        Radial = cols[ridx][:]
        Chord  = cols[cidx][:]
        # optional others left as zeros
        Twist = [0.0]*len(numeric_rows)
        Sweep = [0.0]*len(numeric_rows)
        Anhedral = [0.0]*len(numeric_rows)
        hdr_used = header_tokens or []
    else:
        def get_col_by_idx(name: str, required: bool) -> List[float]:
            if name not in idx_map:
                if required:
                    raise ValueError(f"AERO missing required column '{name}'.")
                warnings.append(f"AERO missing optional column '{name}', filled with 0.")
                return [0.0]*len(numeric_rows)
            j = idx_map[name]
            return [ (row[j] if j < len(row) else 0.0) for row in numeric_rows ]

        Radial = get_col_by_idx("Radial", True)
        Chord  = get_col_by_idx("Chord",  True)
        Twist  = get_col_by_idx("Twist",  False)
        Sweep  = get_col_by_idx("Sweep",  False)
        Anhedral = get_col_by_idx("Anhedral", False)
        hdr_used = header_tokens

    # ------- sort & dedupe by Radial -------
    def _dedupe_and_sort(x: List[float], data: Dict[str, List[float]]):
        x_arr = np.asarray(x, dtype=float)
        order = np.argsort(x_arr)
        x_sorted = x_arr[order]
        out = {k: np.asarray(v, dtype=float)[order] for k, v in data.items()}
        uniq, inv = np.unique(x_sorted, return_inverse=True)
        res = {k: np.zeros_like(uniq, dtype=float) for k in out}
        cnt = np.bincount(inv, minlength=len(uniq))
        for k, arr in out.items():
            res[k] = np.bincount(inv, weights=arr, minlength=len(uniq)) / np.maximum(cnt, 1)
        return uniq.tolist(), {k: v.tolist() for k, v in res.items()}

    r_sorted, cols_sorted = _dedupe_and_sort(
        Radial,
        {"Chord":Chord, "Twist":Twist, "Sweep":Sweep, "Anhedral":Anhedral}
    )

    aero = AeroData(
        Radial = r_sorted,
        Chord  = cols_sorted["Chord"],
        Twist  = cols_sorted["Twist"],
        Sweep  = cols_sorted["Sweep"],
        Anhedral = cols_sorted["Anhedral"],
        meta = {"warnings": warnings, "header": hdr_used, "units_policy": units_policy}
    )
    return aero
