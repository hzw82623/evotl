# main.py
"""
Blade-only generation pipeline with support for single or multi-rotor inputs.

Minimal single-rotor CLI:
  --tip  <path/to/XV_15_rotor.tip>
  --aero <path/to/XV_15_rotor.dat>   # optional
  --out  <path/to/out_dir>

Multi-rotor CLI:
  --rotors-xml <path/to/旋翼.XML>
  --out        <path/to/out_dir>
  [--vehicle-xml <path/to/机身.xml>]

Everything else uses defaults (SI: kg–m–s). See the parameter glossary at the end of this file.
"""

from dataclasses import dataclass
from typing import Optional, List, Iterable
import argparse
import os
import sys

# --- Robust imports: work both as a package module and as a plain script ---
try:
    # package-style (python -m bladegen.main)
    from .io_blade import TipData, AeroData, load_tip, load_aero
    from .select_sections import (
        SectionSelectionConfig,
        SectionReport,
        auto_select_sections,
    )
    from .grid import BladeGrid, build_grid, attach_interpolators
    from .refs_nodes import write_refs, write_nodes
    from .beam import write_beam_file
    from .bodies import write_bodies_file
    from .aero import write_aero_refs_file, write_aero_beam_file
    from .rotors_xml import RotorCfg, parse_rotors_xml, parse_vehicle_xml
    from .gcs import write_gcs_refs
except Exception:
    # script-style (python main.py)
    from io_blade import TipData, AeroData, load_tip, load_aero  # type: ignore
    from select_sections import (  # type: ignore
        SectionSelectionConfig,
        SectionReport,
        auto_select_sections,
    )
    from grid import BladeGrid, build_grid, attach_interpolators  # type: ignore
    from refs_nodes import write_refs, write_nodes  # type: ignore
    from beam import write_beam_file  # type: ignore
    from bodies import write_bodies_file  # type: ignore
    from aero import write_aero_refs_file, write_aero_beam_file  # type: ignore
    from rotors_xml import RotorCfg, parse_rotors_xml, parse_vehicle_xml  # type: ignore
    from gcs import write_gcs_refs  # type: ignore


@dataclass
class MainConfig:
    # Inferred
    name: str
    out_dir: str
    # IO / units (policy tag; real conversion only if implemented in io_blade)
    units_tip: str = "si"      # 'si' (kg–m–s) by default
    units_aero: str = "si"
    # Section selection defaults
    r_start: Optional[float] = None
    err_tol: float = 0.05
    jump_tol: float = 0.10
    max_elems: int = 40
    max_dr: Optional[float] = None
    min_dr: Optional[float] = None
    c_eps: float = 1e-3
    manual_sections: Optional[List[float]] = None
    # Structural
    nu: float = 0.33
    # Behavior
    overwrite: bool = True


def _infer_name_from_tip(tip_path: str) -> str:
    base = os.path.basename(tip_path)
    stem, _ = os.path.splitext(base)
    return "".join(ch if (ch.isalnum() or ch in ("_", "-")) else "_" for ch in stem) or "BLADE"


def _sanitize_name(name: str, fallback: str) -> str:
    cleaned = "".join(ch if (ch.isalnum() or ch in ("_", "-")) else "_" for ch in name)
    cleaned = cleaned.strip("_")
    return cleaned or fallback


def _y_sign_from_direction(direction: str) -> float:
    d = (direction or "").strip().lower()
    if ("顺时" in d) or d == "cw":
        return -1.0
    if d == "顺时针":
        return -1.0
    return 1.0


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    """Parse CLI arguments."""
    ap = argparse.ArgumentParser(
        description="Blade-only generator supporting single or multi-rotor inputs"
    )
    ap.add_argument("--tip", default=None, help="Path to XV-15-style .tip (single-rotor mode)")
    ap.add_argument("--aero", default=None, help="Path to aero .dat (single-rotor mode)")
    ap.add_argument("--out", required=True, help="Output directory")
    ap.add_argument("--rotors-xml", default=None, help="Path to 旋翼.XML (multi-rotor mode)")
    ap.add_argument("--vehicle-xml", default=None, help="Path to 机身.xml (reserved)")
    return ap.parse_args(argv)


def _write_report(
    cfg: MainConfig,
    report: SectionReport,
    extra_lines: Optional[Iterable[str]] = None,
) -> None:
    report_path = os.path.join(cfg.out_dir, "blade.report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("# Section selection report\n")
        f.write(f"name={cfg.name}\n")
        f.write(f"units_tip={cfg.units_tip}, units_aero={cfg.units_aero}\n")
        if extra_lines:
            for line in extra_lines:
                f.write(f"{line}\n")
        f.write(f"K={len(report.sections)}, elems={report.elems}, nodes={report.nodes}\n")
        f.write(f"r_start_used={report.r_start_used}\n")
        f.write(
            f"params: err_tol={cfg.err_tol}, jump_tol={cfg.jump_tol}, max_elems={cfg.max_elems}, "
            f"max_dr={cfg.max_dr}, min_dr={cfg.min_dr}, nu={cfg.nu}, c_eps={cfg.c_eps}\n\n"
        )
        f.write("Reasons per section:\n")
        for r in report.sections:
            f.write(f"  {r:.6f}: {', '.join(report.reasons.get(r, []))}\n")
        if report.warnings:
            f.write("\nWarnings:\n")
            for w in report.warnings:
                f.write(f"  - {w}\n")
        if report.notes:
            f.write("\nNotes:\n")
            for n in report.notes:
                f.write(f"  - {n}\n")


def _run_single_blade(
    cfg: MainConfig,
    tip_path: str,
    aero_path: Optional[str],
    y_sign: float,
    extra_report_lines: Optional[Iterable[str]] = None,
) -> None:
    tip: TipData = load_tip(tip_path, units_policy=cfg.units_tip)
    aero: Optional[AeroData] = (
        load_aero(aero_path, units_policy=cfg.units_aero) if aero_path else None
    )

    sel_cfg = SectionSelectionConfig(
        r_start=cfg.r_start,
        err_tol=cfg.err_tol,
        jump_tol=cfg.jump_tol,
        max_elems=cfg.max_elems,
        max_dr=cfg.max_dr,
        min_dr=cfg.min_dr,
        c_eps=cfg.c_eps,
    )
    sections, report = auto_select_sections(tip, aero, sel_cfg)

    grid: BladeGrid = build_grid(sections)
    grid = attach_interpolators(grid, tip, aero)

    write_refs(tip, grid, cfg.name, os.path.join(cfg.out_dir, "blade.ref"), y_sign=y_sign)
    write_nodes(grid, cfg.name, os.path.join(cfg.out_dir, "blade.nod"))
    write_beam_file(
        tip,
        grid,
        cfg.name,
        cfg.nu,
        os.path.join(cfg.out_dir, "blade.beam"),
        y_sign=y_sign,
    )
    write_bodies_file(tip, grid, cfg.name, os.path.join(cfg.out_dir, "blade.body"))
    write_aero_refs_file(grid, cfg.name, os.path.join(cfg.out_dir, "blade_aero.ref"))
    if aero:
        write_aero_beam_file(
            aero, grid, cfg.name, os.path.join(cfg.out_dir, "blade.aerobeam")
        )

    _write_report(cfg, report, extra_lines=extra_report_lines)


def main(argv: Optional[List[str]] = None) -> int:
    """Run the pipeline; return 0 if OK."""
    args = parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    if args.vehicle_xml:
        # Placeholder parsing to validate presence; result kept for future integration.
        parse_vehicle_xml(args.vehicle_xml)

    if args.rotors_xml:
        try:
            rotors = parse_rotors_xml(args.rotors_xml)
        except (FileNotFoundError, ValueError) as exc:
            print(f"Error loading rotors XML: {exc}")
            return 1
        write_gcs_refs(rotors, os.path.join(args.out, "GCS.ref"))

        for rotor in rotors:
            if not rotor.shape_tip_path:
                print(
                    f"[WARN] Rotor {rotor.index} ('{rotor.name}') missing shape_tip_path; skipped."
                )
                continue

            rotor_name = _sanitize_name(rotor.name or f"rotor_{rotor.index}", f"rotor_{rotor.index}")
            rotor_out = os.path.join(args.out, rotor_name)
            os.makedirs(rotor_out, exist_ok=True)

            cfg = MainConfig(
                name=rotor_name,
                out_dir=rotor_out,
                units_tip="si",
                units_aero="si",
                r_start=None,
                err_tol=0.05,
                jump_tol=0.10,
                max_elems=40,
                max_dr=None,
                min_dr=None,
                c_eps=1e-3,
                manual_sections=None,
                nu=0.33,
                overwrite=True,
            )

            y_sign = _y_sign_from_direction(rotor.direction)
            extra_lines = [
                f"direction={rotor.direction}",
                f"y_sign={y_sign}",
                f"center={rotor.center_xyz}",
            ]
            _run_single_blade(
                cfg,
                rotor.shape_tip_path,
                None,
                y_sign=y_sign,
                extra_report_lines=extra_lines,
            )

        print(f"Done multi-rotor. Outputs in: {args.out}")
        return 0

    if not args.tip:
        raise SystemExit(
            "Provide --tip for single-rotor mode or --rotors-xml for multi-rotor mode."
        )

    cfg = MainConfig(
        name=_infer_name_from_tip(args.tip),
        out_dir=args.out,
        units_tip="si",
        units_aero="si",
        r_start=None,
        err_tol=0.05,
        jump_tol=0.10,
        max_elems=40,
        max_dr=None,
        min_dr=None,
        c_eps=1e-3,
        manual_sections=None,
        nu=0.33,
        overwrite=True,
    )

    _run_single_blade(cfg, args.tip, args.aero, y_sign=1.0)

    print(f"Done single-rotor. Outputs in: {cfg.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""
Parameter Glossary (defaults live inside MainConfig)
---------------------------------------------------
name        : 叶片名/标签。默认从 --tip 文件名自动推断（去后缀）。
out_dir     : 输出目录。

units_tip   : TIP 数据的单位策略标记，默认 'si'（kg–m–s）。目前仅作为标签传入 io_blade；
              若你在 io_blade 里实现了单位换算，则会按此策略进行转换。
units_aero  : AERO 数据的单位策略标记，默认 'si'。

r_start     : 起算半径；None 表示自动识别（优先从 Chord>c_eps 的首个半径；否则从结构量非零处）。
err_tol     : 中点相对误差阈值。误差细化时，若某段对任一信号 (Chord/EA/EJY/EJZ/GJ)
              的中点误差超过该阈值，则在中点插入一个控制截面。
jump_tol    : 跳变检测阈值。相邻采样点相对跃变 |Δ|/max(|·|) ≥ jump_tol 时，会在对应位置
              强制加入控制截面（JUMP 标记）。
max_elems   : 最大单元数（硬上限）。自动细化/强制等分不能超过此上限。
max_dr      : 最大段长。若某段长度 > max_dr，会被等比分割（加入 MAX_DR 标记的截面）。
min_dr      : 最小段长。若出现很短的段，会尝试合并掉非保护截面（保护：START/END/JUMP/VERTEX）。
c_eps       : 用于自动识别 r_start 的弦长阈值（Chord > c_eps 即认为进入有效叶片段）。
nu          : 泊松比，用于 GAy/GAz = EA / [2(1+nu)] 的构造（beam 的 6×6 刚度矩阵）。

manual_sections : 手动给定控制截面（None 表示不启用）。在精简 CLI 中我们未暴露该参数。

overwrite   : 目前未使用；若你将来要做“存在即跳过”，可用它控制写文件行为。
"""
