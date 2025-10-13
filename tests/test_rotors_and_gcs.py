import os
from pathlib import Path

import pytest

pytest.importorskip("numpy")

from gcs import write_gcs_refs
from rotors_xml import parse_rotors_xml, RotorCfg
from refs_nodes import write_refs
from beam import write_beam_file


def test_parse_rotors_xml_resolves_relative_paths(tmp_path: Path):
    tip_dir = tmp_path / "shapes"
    tip_dir.mkdir()
    tip_path = tip_dir / "demo.tip"
    tip_path.write_text("dummy tip", encoding="utf-8")

    xml_content = """
    <旋翼参数>
        <旋翼1>
            <旋翼名称>Rotor-A</旋翼名称>
            <中心点坐标>1.0, 2.0, 3.0</中心点坐标>
            <姿态角>0, 0, 0</姿态角>
            <桨叶片数>4</桨叶片数>
            <转速>600</转速>
            <旋翼旋向>顺时针</旋翼旋向>
            <形状系数>shapes/demo.tip</形状系数>
        </旋翼1>
        <旋翼2>
            <旋翼名称>Rotor-B</旋翼名称>
            <中心点坐标>4,5,6</中心点坐标>
            <姿态角>0,0,0</姿态角>
            <桨叶片数>3</桨叶片数>
            <转速>550</转速>
            <旋翼旋向>逆时针</旋翼旋向>
            <形状系数></形状系数>
        </旋翼2>
    </旋翼参数>
    """
    xml_path = tmp_path / "rotors.xml"
    xml_path.write_text(xml_content, encoding="utf-8")

    rotors = parse_rotors_xml(str(xml_path))
    assert len(rotors) == 2

    first = rotors[0]
    assert first.name == "Rotor-A"
    assert first.center_xyz == pytest.approx((1.0, 2.0, 3.0))
    assert first.shape_tip_path == os.path.abspath(str(tip_path))
    assert "顺时" in first.direction

    second = rotors[1]
    assert second.shape_tip_path is None
    assert "逆时" in second.direction


def test_write_gcs_refs(tmp_path: Path):
    rotors = [
        RotorCfg(
            index=1,
            name="R1",
            center_xyz=(1.0, 0.0, -1.0),
            attitude_rpy=(0.0, 0.0, 0.0),
            blade_count=2,
            rpm=500.0,
            direction="逆时针",
            shape_tip_path=None,
        ),
        RotorCfg(
            index=2,
            name="R2",
            center_xyz=(0.1, 0.2, 0.3),
            attitude_rpy=(0.0, 0.0, 0.0),
            blade_count=3,
            rpm=400.0,
            direction="顺时针",
            shape_tip_path=None,
        ),
    ]
    out_path = tmp_path / "GCS.ref"
    write_gcs_refs(rotors, str(out_path))

    content = out_path.read_text(encoding="utf-8")
    assert "reference: ROTOR_1" in content
    assert "1.0000000000" in content
    assert "0.2000000000" in content


class DummyTip:
    pass


class DummyGrid:
    def __init__(self):
        self.nodes = [0.0]
        self.elements = [(0.0, 0.5, 1.0)]
        self.eval_points = [(0.25, 0.75)]

        data = {
            "ROTAPI_deg": 0.0,
            "YNA": 0.1,
            "ZNA": 0.0,
            "YCG": 0.2,
            "ZCG": 0.0,
            "YCT": 0.3,
            "ZCT": 0.0,
            "EA": 1.0,
            "EJY": 1.0,
            "EJZ": 1.0,
            "GJ": 1.0,
            "ROTAN_deg": 5.0,
        }

        def interp(field: str, _x: float) -> float:
            return data[field]

        self.interp_tip = interp
        self.interp_aero = None


def test_y_sign_mirroring_in_refs_and_beam(tmp_path: Path):
    grid = DummyGrid()
    tip = DummyTip()

    ref_path = tmp_path / "blade.ref"
    write_refs(tip, grid, "TEST", str(ref_path), y_sign=-1.0)
    ref_text = ref_path.read_text(encoding="utf-8")
    assert "-0.1000000000" in ref_text
    assert "-0.2000000000" in ref_text

    beam_path = tmp_path / "blade.beam"
    write_beam_file(tip, grid, "TEST", 0.33, str(beam_path), y_sign=-1.0)
    beam_text = beam_path.read_text(encoding="utf-8")
    assert "-0.2000000000" in beam_text
    assert "linear elastic generic" in beam_text
