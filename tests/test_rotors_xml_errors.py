from pathlib import Path

import pytest

from rotors_xml import parse_rotors_xml


def test_parse_rotors_xml_raises_on_non_xml(tmp_path: Path):
    bogus_path = tmp_path / "fake.tip"
    bogus_path.write_text("not xml content", encoding="utf-8")

    with pytest.raises(ValueError) as excinfo:
        parse_rotors_xml(str(bogus_path))

    assert "Failed to parse rotors XML" in str(excinfo.value)
    assert str(bogus_path) in str(excinfo.value)


def test_main_reports_rotors_xml_error(tmp_path: Path, capsys: pytest.CaptureFixture[str]):
    pytest.importorskip("numpy")
    from main import main as run_main

    bogus_path = tmp_path / "fake.tip"
    bogus_path.write_text("still not xml", encoding="utf-8")
    out_dir = tmp_path / "out"

    exit_code = run_main(["--out", str(out_dir), "--rotors-xml", str(bogus_path)])
    captured = capsys.readouterr()

    assert exit_code == 1
    assert "Error loading rotors XML" in captured.out
    assert str(bogus_path) in captured.out
