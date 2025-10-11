import math

import pytest

pytest.importorskip("numpy")

from grid import build_grid


def test_build_grid_structure_and_eval_points():
    sections = [0.0, 0.5, 1.5]
    grid = build_grid(sections)

    assert grid.sections == pytest.approx(sections)
    assert grid.nodes == pytest.approx([0.0, 0.25, 0.5, 1.0, 1.5])
    expected_elements = [(0.0, 0.25, 0.5), (0.5, 1.0, 1.5)]
    for element, expected in zip(grid.elements, expected_elements):
        assert element == pytest.approx(expected)

    inv_sqrt3 = 1.0 / math.sqrt(3.0)
    expected_eval = [
        (0.25 - 0.5 * inv_sqrt3 * 0.5, 0.25 + 0.5 * inv_sqrt3 * 0.5),
        (1.0 - 0.5 * inv_sqrt3 * 1.0, 1.0 + 0.5 * inv_sqrt3 * 1.0),
    ]
    for pair, expected in zip(grid.eval_points, expected_eval):
        assert pair == pytest.approx(expected)

    assert grid.interp_tip is None
    assert grid.interp_aero is None
