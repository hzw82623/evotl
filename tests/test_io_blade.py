import textwrap

import pytest

pytest.importorskip("numpy")

from io_blade import load_aero


def test_load_aero_single_column_error(tmp_path):
    content = textwrap.dedent(
        """
        0.1
        0.2
        0.3
        """
    ).strip()
    aero_path = tmp_path / "single_column.dat"
    aero_path.write_text(content, encoding="utf-8")

    with pytest.raises(ValueError) as excinfo:
        load_aero(str(aero_path))

    assert "could not infer Radial/Chord columns" in str(excinfo.value)
