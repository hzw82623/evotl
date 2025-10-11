# rotors_xml.py
"""Parsers for multi-rotor XML inputs."""
from dataclasses import dataclass
from typing import List, Optional, Tuple
import xml.etree.ElementTree as ET
import os
import re


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


@dataclass
class VehicleCfg:
    """Placeholder configuration for vehicle-level data (reserved)."""
    raw_path: Optional[str] = None


def _fix_malformed_tags(xml_text: str) -> str:
    """Fix known malformed tags in the provided XML text."""
    return re.sub(r"<形状系数>([^<]+)<形状系数>", r"<形状系数>\\1</形状系数>", xml_text)


def _get_text(elem: Optional[ET.Element]) -> str:
    return (elem.text or "").strip() if elem is not None else ""


def _to_floats_csv(value: str) -> Tuple[float, float, float]:
    parts = [p.strip() for p in value.replace("，", ",").split(",") if p.strip()]
    vals = [float(parts[i]) if i < len(parts) else 0.0 for i in range(3)]
    return (vals[0], vals[1], vals[2])


def parse_rotors_xml(xml_path: str) -> List[RotorCfg]:
    """Parse the multi-rotor description XML."""
    if not os.path.isfile(xml_path):
        raise FileNotFoundError(f"Rotors XML not found: {xml_path}")

    with open(xml_path, "r", encoding="utf-8", errors="ignore") as fh:
        xml_text = _fix_malformed_tags(fh.read())

    root = ET.fromstring(xml_text)
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

        shape_path = (
            _get_text(rotor_node.find("桨叶剖面/结构剖面/形状系数"))
            or _get_text(rotor_node.find("形状系数"))
            or None
        )
        if shape_path and not os.path.isabs(shape_path):
            shape_path = os.path.abspath(os.path.join(os.path.dirname(xml_path), shape_path))

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
            )
        )

    return rotors


def parse_vehicle_xml(xml_path: str) -> VehicleCfg:
    """Placeholder parser for future vehicle-level data."""
    return VehicleCfg(raw_path=xml_path)
