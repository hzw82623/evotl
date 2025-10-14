"""Helpers to assemble the project-level main.mbd input file."""

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


def _fe(x: float) -> str:
    return f"{x:.6e}"


def _path_rel(from_dir: str, to_path: str) -> str:
    return os.path.normpath(os.path.relpath(to_path, start=from_dir)).replace("\\", "/")


def _safe_exists(path: str) -> bool:
    try:
        return os.path.isfile(path)
    except Exception:
        return False


def _count_token_lines(path: str, token: str) -> int:
    if not _safe_exists(path):
        return 0
    count = 0
    with open(path, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            if token in line:
                count += 1
    return count


@dataclass
class RotorOut:
    """Per-rotor output metadata needed to assemble the main file."""

    index: int
    name: str
    out_dir: str
    blade_count: int
    has_aero: bool


@dataclass
class SimParams:
    """Simulation controls for the initial value block."""

    time_step: float = 1.0e-3
    t0: float = 0.0
    t_end: float = 10.0
    max_iters: int = 100
    tol: float = 1.0e-1
    deriv_tol: float = 1.0e38
    deriv_max_iters: int = 30
    deriv_coeff: float = 1.0
    linear_solver: str = "naive, colamd, pivot factor, 1.e0"
    method: str = "ms, 0.2"


@dataclass
class ControlVars:
    """Control-data counters (blade related auto-calculated)."""

    num_blade_nodes: int
    num_blade_bodies: int
    num_beam: int
    num_aerobeam: int
    num_rotor_static_nodes: int = 0
    num_rotor_dynamic_bodies: int = 0
    num_force: int = 0
    num_joints_rotor: int = 0
    num_joints_blade_yoke: int = 0


def _scan_blade_counts(rotor_dirs: List[RotorOut]) -> ControlVars:
    n_nodes = 0
    n_bodies = 0
    n_beams = 0
    n_aero = 0
    for rotor in rotor_dirs:
        count = getattr(rotor, "blade_count", 1) or 1
        path_nod = os.path.join(rotor.out_dir, "blade.nod")
        path_beam = os.path.join(rotor.out_dir, "blade.beam")
        path_body = os.path.join(rotor.out_dir, "blade.body")
        path_aero = os.path.join(rotor.out_dir, "blade.aerobeam")

        n_nodes += _count_token_lines(path_nod, "structural:") * count
        n_bodies += _count_token_lines(path_body, "body:") * count
        n_beams += _count_token_lines(path_beam, "beam3:") * count
        n_aero += _count_token_lines(path_aero, "aerodynamic beam3:") * count

    return ControlVars(
        num_blade_nodes=n_nodes,
        num_blade_bodies=n_bodies,
        num_beam=n_beams,
        num_aerobeam=n_aero,
    )


def write_main_mbd(
    project_out_dir: str,
    rotor_outputs: List[RotorOut],
    sim: Optional[SimParams] = None,
    ctrl_override: Optional[Dict[str, int]] = None,
    c81_pairs: Optional[List[Tuple[str, str]]] = None,
    extra_includes_before_nodes: Optional[List[str]] = None,
    extra_elements_lines: Optional[List[str]] = None,
    theta_coll_deg: float = 5.0,
) -> str:
    """Write main.mbd under ``project_out_dir`` and return the file path."""

    os.makedirs(project_out_dir, exist_ok=True)
    sim = sim or SimParams()

    expanded: List[RotorOut] = []
    for rotor in rotor_outputs:
        for _ in range(max(1, rotor.blade_count)):
            expanded.append(RotorOut(rotor.index, rotor.name, rotor.out_dir, 1, rotor.has_aero))
    ctrl = _scan_blade_counts(expanded)
    if ctrl_override:
        for key, value in ctrl_override.items():
            if hasattr(ctrl, key):
                setattr(ctrl, key, int(value))

    lines: List[str] = []
    lines += [
        "begin: data;",
        "    problem: initial value;",
        "end: data;",
        "",
    ]

    lines += [
        "begin: initial value;",
        f"    time step: {_fe(sim.time_step)};",
        f"    initial time: {sim.t0:.6f};",
        f"    final time: {sim.t_end:.6f} ;",
        f"    max iterations: {sim.max_iters};",
        f"    tolerance: {_fe(sim.tol)};",
        f"    derivatives tolerance: {_fe(sim.deriv_tol)};",
        f"    derivatives max iterations: {sim.deriv_max_iters};",
        f"    derivatives coefficient: {_fe(sim.deriv_coeff)};",
        f"    linear solver: {sim.linear_solver};",
        f"    method: {sim.method};",
        "    #output: residual;",
        "    output: counter;",
        "    /*",
        "    eigenanalysis: 0.10000,",
        "        output matrices,",
        "        output eigenvectors,",
        "        output geometry,",
        "        upper frequency limit, 400,",
        "        lower frequency limit, 1,",
        "        use lapack;*/",
        "end: initial value;",
        "",
    ]

    lines += [
        "# CONTROL DATA SECTION",
        "begin: control data;",
        f"set: const integer num_blade        = {ctrl.num_blade_nodes};",
        f"set: const integer num_blade_dynamic= {ctrl.num_blade_bodies};",
        f"set: const integer num_beam         = {ctrl.num_beam};",
        f"set: const integer num_aerobeam     = {ctrl.num_aerobeam};",
        f"set: const integer num_rotor        = {ctrl.num_rotor_static_nodes};",
        f"set: const integer num_rotor_dynamic= {ctrl.num_rotor_dynamic_bodies};",
        f"set: const integer num_force        = {ctrl.num_force};",
        f"set: const integer num_joint_rotor  = {ctrl.num_joints_rotor};",
        f"set: const integer num_joint_by     = {ctrl.num_joints_blade_yoke};",
        "",
        "    structural nodes:",
        "        num_blade +",
        "        num_rotor",
        "        ;",
        "    rigid bodies:",
        "        num_blade_dynamic +",
        "        num_rotor_dynamic",
        "        ;",
        "    aerodynamic elements:",
        "        num_aerobeam",
        "        ;",
        "    forces: num_force;",
        "",
        "    beams:",
        "        num_beam +",
        "        ;",
        "    joints:",
        "        num_blade +",
        "        num_rotor",
        "        ;",
        "    inertia: 1;",
        "    air properties;",
        "    gravity;",
        "    induced velocity elements:+1",
        "    ;",
        "    output results: netcdf;",
        "    default output: reference frames;",
        "    default orientation: orientation vector;",
        "    print: equation description;",
        "end: control data;",
        "",
    ]

    if c81_pairs:
        for name, rel in c81_pairs:
            lines.append(f'c81 data: {name}, "{rel}";')
        lines.append("")

    lines += [
        "set: const integer CURR_ROTOR = 0;",
        "set: const integer CURR_blade = 0;",
        "",
        '# Reference frame',
        'include: "GCS.ref";',
        "",
        "# Blades (refs for each blade copy)",
    ]

    for rotor in rotor_outputs:
        rotor_base = rotor.index * 100000
        rel_ref = _path_rel(project_out_dir, os.path.join(rotor.out_dir, "blade.ref"))
        for blade_idx in range(1, max(1, rotor.blade_count) + 1):
            blade_base = blade_idx * 10000
            lines.append(f"set: CURR_ROTOR = {rotor_base};")
            lines.append(f"set: CURR_blade = {blade_base};")
            lines.append(f'include: "{rel_ref}";')

    if extra_includes_before_nodes:
        lines += [ln.rstrip() for ln in extra_includes_before_nodes]
        lines.append("")

    lines += ["", "begin: nodes;", ""]

    for rotor in rotor_outputs:
        rotor_base = rotor.index * 100000
        rel_nod = _path_rel(project_out_dir, os.path.join(rotor.out_dir, "blade.nod"))
        for blade_idx in range(1, max(1, rotor.blade_count) + 1):
            blade_base = blade_idx * 10000
            lines.append(f"    set: CURR_ROTOR = {rotor_base};")
            lines.append(f"    set: CURR_blade = {blade_base};")
            lines.append(f'    include: "{rel_nod}";')
        lines.append("")

    lines += ["end: nodes;", "", "begin: elements;", ""]

    for rotor in rotor_outputs:
        rotor_base = rotor.index * 100000
        rel_beam = _path_rel(project_out_dir, os.path.join(rotor.out_dir, "blade.beam"))
        rel_body = _path_rel(project_out_dir, os.path.join(rotor.out_dir, "blade.body"))
        path_aero = os.path.join(rotor.out_dir, "blade.aerobeam")
        rel_aero = _path_rel(project_out_dir, path_aero)
        has_aero = _safe_exists(path_aero)
        lines.append(f"    # --- {rotor.name} ---")
        for blade_idx in range(1, max(1, rotor.blade_count) + 1):
            blade_base = blade_idx * 10000
            lines.append(f"    set: CURR_ROTOR = {rotor_base};")
            lines.append(f"    set: CURR_blade = {blade_base};")
            lines.append(f'    include: "{rel_beam}";')
            lines.append(f'    include: "{rel_body}";')
            if has_aero:
                lines.append(f'    include: "{rel_aero}";')
        lines.append("")

    lines += [
        "    air properties: 1.225, 340.0,",
        "        -1., 0., 0., const,0;",
        "        #cosine, 15., pi/10, WIND_SPEED/2, half, 0.;",
        "",
    ]

    if extra_elements_lines:
        lines += [ln.rstrip() for ln in extra_elements_lines]
        lines.append("")

    lines += ["end: elements;", ""]

    out_path = os.path.join(project_out_dir, "main.mbd")
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    return out_path
