# rotors_xml.py
"""Parsers for multi-rotor XML inputs."""
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET
import os
import re

from io_blade import AeroData


@dataclass
class RotorCfg:
    """Configuration extracted for a single rotor."""
    index: int
    name: str
    center_xyz: Tuple[float, float, float]
    attitude_rpy: Tuple[float, float, float]
    blade_count: int
    rpm: float
    direction: str
    shape_tip_path: Optional[str]
    aero_start_xyz: Optional[Tuple[float, float, float]] = None
    aero_data_block: Optional[str] = None


@dataclass
class VehicleCfg:
    """Placeholder configuration for vehicle-level data (reserved)."""
    raw_path: Optional[str] = None


def _fix_malformed_tags(xml_text: str) -> str:
    """Fix known malformed tags in the provided XML text."""

    return re.sub(r"<形状系数>([^<]+)<形状系数>", r"<形状系数>\1</形状系数>", xml_text)


def _looks_windows_absolute(path: str) -> bool:
    """Return ``True`` if *path* looks like a Windows absolute path."""

    if not path:
        return False
    if path.startswith("\\\\") or path.startswith("//"):
        return True
    return bool(re.match(r"^[A-Za-z]:[\\/].*", path))


def _resolve_tip_path(xml_path: str, raw_path: Optional[str]) -> Optional[str]:
    """Resolve a tip path, handling relative paths and Windows absolutes."""

    if not raw_path:
        return None

    xml_dir = os.path.dirname(os.path.abspath(xml_path))
    candidate = raw_path.strip()
    candidate = candidate.replace("\\", os.sep)

    looks_abs = os.path.isabs(candidate) or _looks_windows_absolute(raw_path)
    if looks_abs and os.path.isfile(candidate):
        return os.path.abspath(candidate)

    if not looks_abs:
        rel_candidate = os.path.abspath(os.path.join(xml_dir, candidate))
        if os.path.isfile(rel_candidate):
            return rel_candidate

    base_name = os.path.basename(candidate)
    fallback = os.path.abspath(os.path.join(xml_dir, base_name))
    if os.path.isfile(fallback):
        return fallback

    if looks_abs:
        return os.path.normpath(candidate)

    return os.path.abspath(candidate)


def _get_text(elem: Optional[ET.Element]) -> str:
    return (elem.text or "").strip() if elem is not None else ""


def _to_floats_csv(value: str) -> Tuple[float, float, float]:
    parts = [p.strip() for p in value.replace("，", ",").split(",") if p.strip()]
    vals = [float(parts[i]) if i < len(parts) else 0.0 for i in range(3)]
    return (vals[0], vals[1], vals[2])


@dataclass
class AeroSeg:
    chord: float
    twist_deg: float
    c81: Optional[str]
    dr: Optional[float]
    sweep: float
    anhedral: float
    divs: Optional[int]
    div_type: Optional[str]


def _parse_float_safe(value: str, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def parse_aero_segments(csv_block: str) -> List[AeroSeg]:
    """Parse the aero CSV block from the XML into segments."""

    text = (csv_block or "").strip()
    if not text:
        return []

    rows = re.split(r"[;；]\s*", text)
    segments: List[AeroSeg] = []
    for row in rows:
        if not row:
            continue
        cols = [c.strip() for c in row.replace("，", ",").split(",")]
        while len(cols) < 9:
            cols.append("")
        chord = _parse_float_safe(cols[1])
        twist = _parse_float_safe(cols[2])
        c81 = cols[3] or None
        dr = float(cols[4]) if cols[4] else None
        sweep = _parse_float_safe(cols[5])
        anhedral = _parse_float_safe(cols[6])
        divs = int(float(cols[7])) if cols[7] else None
        div_type = cols[8] or None
        segments.append(AeroSeg(chord, twist, c81, dr, sweep, anhedral, divs, div_type))

    return segments


def build_aerodata_from_rotor_xml(
    wing_start_xyz: Tuple[float, float, float], csv_block: str
) -> Optional[AeroData]:
    """Convert rotor XML aero data into :class:`AeroData` using v1.1 rules."""

    segments = parse_aero_segments(csv_block)
    if not segments:
        return None

    r0 = float(wing_start_xyz[1]) if wing_start_xyz else 0.0
    radial: List[float] = []
    chord: List[float] = []
    twist: List[float] = []
    sweep: List[float] = []
    anhedral: List[float] = []

    acc = r0
    for idx, seg in enumerate(segments):
        if idx == 0:
            acc = r0
        else:
            prev = segments[idx - 1]
            if prev.dr is not None:
                acc += max(0.0, prev.dr)
        radial.append(acc)
        chord.append(seg.chord)
        twist.append(seg.twist_deg)
        sweep.append(seg.sweep)
        anhedral.append(seg.anhedral)

    meta: Dict[str, object] = {
        "source": "xml",
        "c81": [seg.c81 for seg in segments],
        "divisions": [seg.divs for seg in segments],
        "division_type": [seg.div_type for seg in segments],
    }

    return AeroData(
        Radial=radial,
        Chord=chord,
        Twist=twist,
        Sweep=sweep,
        Anhedral=anhedral,
        meta=meta,
    )


def parse_rotors_xml(xml_path: str) -> List[RotorCfg]:
    """Parse the multi-rotor description XML."""
    if not os.path.isfile(xml_path):
        raise FileNotFoundError(f"Rotors XML not found: {xml_path}")

    with open(xml_path, "r", encoding="utf-8", errors="ignore") as fh:
        xml_text = _fix_malformed_tags(fh.read())

    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as exc:
        raise ValueError(
            f"Failed to parse rotors XML '{xml_path}'. Ensure the file is a valid XML document."
        ) from exc
    rotors: List[RotorCfg] = []

    for idx, rotor_node in enumerate(list(root), start=1):
        name = _get_text(rotor_node.find("旋翼名称")) or f"rotor_{idx}"
        center = _to_floats_csv(_get_text(rotor_node.find("中心点坐标")))
        rpy = _to_floats_csv(_get_text(rotor_node.find("姿态角")))

        try:
            blade_count = int(float(_get_text(rotor_node.find("桨叶片数"))))
        except Exception:
            blade_count = 2
        try:
            rpm = float(_get_text(rotor_node.find("转速")))
        except Exception:
            rpm = 0.0

        direction = (
            _get_text(rotor_node.find("桨叶剖面/气动剖面/旋翼旋向"))
            or _get_text(rotor_node.find("旋翼旋向"))
            or "逆时针"
        )

        shape_path_raw = (
            _get_text(rotor_node.find("桨叶剖面/结构剖面/形状系数"))
            or _get_text(rotor_node.find("形状系数"))
            or None
        )
        shape_path = _resolve_tip_path(xml_path, shape_path_raw)

        aero_start_node = rotor_node.find("桨叶剖面/气动剖面/翼型起始位置")
        aero_data_node = rotor_node.find("桨叶剖面/气动剖面/气动数据")
        aero_start_text = _get_text(aero_start_node) if aero_start_node is not None else ""
        aero_start = _to_floats_csv(aero_start_text) if aero_start_text else None
        aero_block = _get_text(aero_data_node) if aero_data_node is not None else None

        rotors.append(
            RotorCfg(
                index=idx,
                name=name,
                center_xyz=center,
                attitude_rpy=rpy,
                blade_count=blade_count,
                rpm=rpm,
                direction=direction,
                shape_tip_path=shape_path,
                aero_start_xyz=aero_start,
                aero_data_block=aero_block,
            )
        )

    return rotors


def parse_vehicle_xml(xml_path: str) -> VehicleCfg:
    """Placeholder parser for future vehicle-level data."""
    return VehicleCfg(raw_path=xml_path)
