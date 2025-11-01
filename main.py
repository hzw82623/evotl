"""Multi-rotor blade generator (XML-driven)."""

import argparse
import os
import sys
from dataclasses import dataclass
from typing import Iterable, List, Optional

from aero import write_aero_beam_file, write_aero_refs_file
from beam import write_beam_file
from bodies import write_bodies_file
from gcs import write_gcs_refs
from grid import BladeGrid, attach_interpolators, build_grid
from io_blade import AeroData, TipData, load_tip
from mbd_writer import RotorOut, SimParams, write_main_mbd
from gen_frameargu import generate as gen_frameargu
from gen_motor import generate as gen_motor
from refs_nodes import write_nodes, write_refs
from rotors_xml import (
    RotorCfg,
    build_aerodata_from_rotor_xml,
    parse_rotors_xml,
)
from select_sections import (
    SectionReport,
    SectionSelectionConfig,
    auto_select_sections,
)
# ==== Global switches (temporary; replace with GlobalSettings.xml later) ====
# aero_mode: "mbdyn" -> generate blade_aero.ref / blade.aerobeam and include them
#            "coupled" -> skip aerodynamic outputs and excludes from main.mbd
AERO_MODE = "mbdyn"


@dataclass
class GenerationConfig:
    name: str
    out_dir: str
    units_tip: str = "si"
    units_aero: str = "si"
    r_start: Optional[float] = None
    err_tol: float = 0.05
    jump_tol: float = 0.10
    max_elems: int = 40
    max_dr: Optional[float] = None
    min_dr: Optional[float] = None
    c_eps: float = 1e-3
    nu: float = 0.33


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


def _write_report(cfg: GenerationConfig, report: SectionReport, extra: Optional[Iterable[str]] = None) -> None:
    out_path = os.path.join(cfg.out_dir, "blade.report.txt")
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write("# Section selection report\n")
        fh.write(f"name={cfg.name}\n")
        fh.write(f"units_tip={cfg.units_tip}, units_aero={cfg.units_aero}\n")
        if extra:
            for line in extra:
                fh.write(f"{line}\n")
        fh.write(
            f"K={len(report.sections)}, elems={report.elems}, nodes={report.nodes}\n"
            f"r_start_used={report.r_start_used}\n"
        )
        fh.write(
            "params: "
            f"err_tol={cfg.err_tol}, jump_tol={cfg.jump_tol}, max_elems={cfg.max_elems}, "
            f"max_dr={cfg.max_dr}, min_dr={cfg.min_dr}, nu={cfg.nu}, c_eps={cfg.c_eps}\n\n"
        )
        fh.write("Reasons per section:\n")
        for r in report.sections:
            fh.write(f"  {r:.6f}: {', '.join(report.reasons.get(r, []))}\n")
        if report.warnings:
            fh.write("\nWarnings:\n")
            for w in report.warnings:
                fh.write(f"  - {w}\n")
        if report.notes:
            fh.write("\nNotes:\n")
            for n in report.notes:
                fh.write(f"  - {n}\n")


def _run_single_blade(
    cfg: GenerationConfig,
    tip_path: str,
    aero_data: Optional[AeroData],
    y_sign: float,
    extra_report_lines: Optional[Iterable[str]] = None,
) -> None:
    tip: TipData = load_tip(tip_path, units_policy=cfg.units_tip)
    aero = aero_data

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
    write_beam_file(tip, grid, cfg.name, cfg.nu, os.path.join(cfg.out_dir, "blade.beam"), y_sign=y_sign)
    write_bodies_file(tip, grid, cfg.name, os.path.join(cfg.out_dir, "blade.body"))
    write_aero_refs_file(grid, cfg.name, os.path.join(cfg.out_dir, "blade_aero.ref"))
    if AERO_MODE == "mbdyn" and aero is not None:
        write_aero_beam_file(aero, grid, cfg.name, os.path.join(cfg.out_dir, "blade.aerobeam"))

    _write_report(cfg, report, extra_report_lines)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Multi-rotor blade generator (XML-only)")
    parser.add_argument("--rotors-xml", required=True, help="Path to 旋翼.XML")
    parser.add_argument("--out", required=True, help="Output directory")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    os.makedirs(args.out, exist_ok=True)

    try:
        rotors: List[RotorCfg] = parse_rotors_xml(args.rotors_xml)
    except Exception as exc:  # noqa: BLE001 - surface parsing issues to the user
        print(f"Error loading rotors XML '{args.rotors_xml}': {exc}")
        return 1
    if not rotors:
        print(f"[WARN] No rotors defined in {args.rotors_xml}")

    write_gcs_refs(rotors, os.path.join(args.out, "GCS.ref"))

    rotor_outputs: List[RotorOut] = []
    for rotor in rotors:
        if not rotor.shape_tip_path:
            print(f"[WARN] Rotor {rotor.index} '{rotor.name}' missing shape (.tip); skipping.")
            continue

        rotor_name = _sanitize_name(rotor.name, f"rotor_{rotor.index}")
        rotor_dir = os.path.join(args.out, rotor_name)
        os.makedirs(rotor_dir, exist_ok=True)

        cfg = GenerationConfig(name=rotor_name, out_dir=rotor_dir)
        y_sign = _y_sign_from_direction(rotor.direction)
        aero_data: Optional[AeroData] = None
        if AERO_MODE == "mbdyn" and rotor.aero_start_xyz and rotor.aero_data_block:
            aero_data = build_aerodata_from_rotor_xml(rotor.aero_start_xyz, rotor.aero_data_block)
        extra_lines = [
            f"rotor_index={rotor.index}",
            f"blade_count={rotor.blade_count}",
            f"direction={rotor.direction}",
            f"aero_mode={AERO_MODE}",
        ]
        _run_single_blade(
            cfg,
            rotor.shape_tip_path,
            aero_data,
            y_sign,
            extra_report_lines=extra_lines,
        )

        has_aero = (
            AERO_MODE == "mbdyn"
            and os.path.isfile(os.path.join(rotor_dir, "blade.aerobeam"))
        )
        rotor_outputs.append(
            RotorOut(
                index=rotor.index,
                name=rotor_name,
                out_dir=rotor_dir,
                blade_count=max(1, rotor.blade_count),
                has_aero=has_aero,
            )
        )

    xml_dir = os.path.dirname(args.rotors_xml)
    frame_node_includes, frame_element_includes = gen_frameargu(
        aer_frameargu_xml=os.path.join(xml_dir, "aer_frameargu.xml"),
        out_dir=args.out,
    )
    motor_node_includes, motor_element_includes = gen_motor(
        act_motor_xml=os.path.join(xml_dir, "act_motor.xml"),
        out_dir=args.out,
    )

    extra_node_includes: List[str] = frame_node_includes + motor_node_includes
    extra_element_includes: List[str] = frame_element_includes + motor_element_includes

    write_main_mbd(
        project_out_dir=args.out,
        rotor_outputs=rotor_outputs,
        sim=SimParams(),
        include_aero=(AERO_MODE == "mbdyn"),
        extra_node_includes=extra_node_includes,
        extra_element_includes=extra_element_includes,
    )
    print(f"Done. Outputs in: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
