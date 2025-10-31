from __future__ import annotations

import os
from typing import Tuple


def parse_triplet(csv_text: str, *, default: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Tuple[float, float, float]:
    """Parse "a,b,c" text into a float triplet, falling back to ``default`` when invalid."""

    try:
        xs = [t.strip() for t in (csv_text or "").split(",")]
        if len(xs) != 3:
            return default
        return float(xs[0]), float(xs[1]), float(xs[2])
    except Exception:
        return default


def write_text(path: str, content: str) -> None:
    """Write *content* to *path*, creating parent directories as needed."""

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(content)


def rel_include_line(file_path: str, start_dir: str, indent: int = 4) -> str:
    """Return an ``include: "...";`` line pointing to *file_path* relative to *start_dir*."""

    rel = os.path.relpath(file_path, start=start_dir).replace("\\", "/")
    return f'{" " * indent}include: "{rel}";'
