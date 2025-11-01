# 模块说明（完整版）

本文档对当前仓库中每一个 Python 模块的用途、重要类型、函数签名、输入输出、副作用以及调用关系进行逐一说明，力求让新的贡献者或 AI 助手在不了解上下文的情况下，也能快速理解体系结构并正确扩展功能。

## 总体架构

代码基于 **旋翼（rotor）→ 桨叶（blade）→ MBDyn 主模型** 的处理流水线构建。核心流程如下：

1. **命令行入口**：`main.py` 解析用户提供的 `--rotors-xml`（旋翼配置 XML）与 `--out`（输出目录）。
2. **旋翼解析**：`rotors_xml.parse_rotors_xml()` 读取 XML，生成多个 `RotorCfg` 对象，并为每个旋翼提供结构 TIP、气动数据等引用。
3. **桨叶生成**：`main._run_single_blade()` 依次处理旋翼的每个桨叶，调用 `select_sections`, `grid`, `refs_nodes`, `beam`, `bodies`, `aero` 等模块，写出桨叶相关文件。
4. **机身/电机生成**：`gen_frameargu.generate()` 与 `gen_motor.generate()` 根据 XML/配置生成 `frameargu.*` 与 `motor.*` 文件，并返回需要 include 到 `main.mbd` 的行。
5. **主装配文件**：`mbd_writer.write_main_mbd()` 汇总所有旋翼与额外组件，生成最终的 `main.mbd`。

下文按模块展开。

---

## 顶层入口：`main.py`

| 项                 | 说明 |
|--------------------|------|
| 主要职责           | 命令行入口； orchestrate 全部子模块。|
| 核心数据结构       | `GenerationConfig`（桨叶生成配置 dataclass），`RotorOut`（由 `mbd_writer` 定义，用于汇总输出信息）。|
| 生成文件（副作用） | `blade.ref/.nod/.beam/.body/.aerobeam/.report`, `frameargu.*`, `motor.*`, `main.mbd` 等。|

### 函数与行为

* `parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace`
  * 解析命令行参数；约束 `--rotors-xml` 和 `--out` 必填。未提供 `argv` 时读取 `sys.argv[1:]`。
* `_run_single_blade(cfg: GenerationConfig, tip_path: str, aero_data: Optional[AeroData], y_sign: float, extra_report_lines: Optional[Iterable[str]] = None) -> RotorOut`
  * 读取桨叶 TIP 数据 (`io_blade.load_tip`)，可选气动数据，选择控制截面 (`select_sections.auto_select_sections`)；
  * 构造网格 (`grid.build_grid` + `grid.attach_interpolators`)，写入参考系 (`refs_nodes.write_refs`)、节点 (`refs_nodes.write_nodes`)、梁 (`beam.write_beam_file`)、质量 (`bodies.write_bodies_file`)、气动 (`aero.write_aero_refs_file`、`aero.write_aero_beam_file`)；
  * 返回 `RotorOut`，其中含桨叶名称、节点计数、生成文件路径等。
* `main(argv: Optional[List[str]] = None) -> int`
  * 入口函数；整体执行顺序：
    1. 调 `parse_args` 获取参数。
    2. 用 `rotors_xml.parse_rotors_xml` 得到 `RotorCfg` 列表。
    3. 对每个 `RotorCfg` 调 `_run_single_blade`（双桨叶时两侧各跑一次）。
    4. 调 `gen_frameargu.generate` 与 `gen_motor.generate` 获取额外 include 行。
    5. 调 `mbd_writer.write_main_mbd` 生成主文件。
    6. 返回 `0`；过程中如遇异常会抛出，交给 `__main__` 逻辑处理。

---

## 通用 I/O 工具：`io_utils.py`

* `parse_triplet(csv_text: str, *, default=(0.0, 0.0, 0.0)) -> Tuple[float, float, float]`
  * 将形如 `"a,b,c"` 的文本解析为浮点三元组；解析失败返回 `default`。
* `write_text(path: str, content: str) -> None`
  * 创建父目录（如果不存在），按 UTF-8 写入文本内容。
* `rel_include_line(file_path: str, start_dir: str, indent: int = 4) -> str`
  * 生成用于 MBDyn 文件的 `include: "rel/path";` 行，路径相对于 `start_dir`，并自动处理 Windows 分隔符。

---

## XML/数据解析层

### `rotors_xml.py`

| 项 | 说明 |
|----|------|
| 职责 | 解析旋翼配置 XML，组装结构/气动数据引用。|
| 主要类型 | `RotorCfg`, `VehicleCfg`, `AeroSeg`。|
| 依赖 | `io_blade.AeroData`, `dataclasses`, `xml.etree.ElementTree`。|

* `parse_rotors_xml(xml_path: str) -> List[RotorCfg]`
  * 输入：旋翼配置 XML 路径。
  * 输出：`RotorCfg` 列表，每个包含桨叶名、TIP 路径、安装位置、旋向、气动段落等。
  * 副作用：检查外部文件存在性（TIP、气动 CSV 等）；若缺失会抛异常。
* `build_aerodata_from_rotor_xml(origin: Tuple[float, float, float], csv_block: Optional[str]) -> Optional[AeroData]`
  * 将 XML 中的嵌入式 CSV 数据转换为 `AeroData`；若缺失或为空则返回 `None`。

### `io_blade.py`

* **数据类**
  * `TipData`: 包含桨叶半径、各控制截面属性（弦长、扭转、质心等）。
  * `AeroData`: 包含气动截面位置、升阻曲线索引、扭转、前掠、下反等。
* **函数**
  * `load_tip(path: str, units_policy: str = "si") -> TipData`
    * 读取 TIP 文件，转换为 SI 单位（默认）。
  * `load_aero(path: str, units_policy: str = "raw") -> Optional[AeroData]`
    * 读取 DAT/CSV 气动文件；若存在缺失列尝试补全；找不到文件时返回 `None` 或抛异常（取决于调用方）。

---

## 桨叶截面选择与网格构建

### `select_sections.py`

* `SectionSelectionConfig`
  * 字段：`r_start`, `err_tol`, `jump_tol`, `max_elems`, `max_dr`, `min_dr`, `c_eps` 等。
  * 描述：控制截面选择算法的阈值，确保剖分精度与最大单元长度。
* `SectionReport`
  * 包含：`selected_sections`, `error_metrics`, `messages` 等，用于输出报告。
* `auto_select_sections(tip: TipData, aero: Optional[AeroData], cfg: SectionSelectionConfig) -> Tuple[List[float], SectionReport]`
  * 输入：结构 TIP 数据、气动数据（可选）、算法配置。
  * 输出：选定的半径列表与报告对象。
  * 副作用：无；纯计算。

### `grid.py`

* `BladeGrid`
  * 字段：`nodes`, `elements`, `span_positions`, `interp_tip`, `interp_aero` 等。
* `build_grid(sections: Iterable[float]) -> BladeGrid`
  * 输入：截面位置；输出：包含节点、梁单元的初始网格。
* `attach_interpolators(grid: BladeGrid, tip: TipData, aero: Optional[AeroData]) -> BladeGrid`
  * 将 TIP/气动数据绑定到网格的插值器上，供后续模块使用。

---

## 桨叶文件写入模块

| 模块 | 主要函数 | 输出文件 | 说明 |
|------|----------|----------|------|
| `refs_nodes.py` | `write_refs(tip, grid, name, out_path, y_sign=1.0)` | `blade.ref` | 定义桨叶局部参考系。|
|                  | `write_nodes(grid, name, out_path)` | `blade.nod` | 写结构节点坐标与参考系指向。|
| `beam.py`       | `write_beam_file(tip, grid, name, nu, out_path, y_sign=1.0)` | `blade.beam` | 生成 `beam3` 元素，依赖 `TipData` 中的刚度矩阵。|
| `bodies.py`     | `write_bodies_file(tip, grid, name, out_path)` | `blade.body` | 计算集中质量、质心、转动惯量。|
| `aero.py`       | `write_aero_refs_file(grid, name, out_path)` | `blade_aero.ref` | 生成气动参考系。|
|                 | `write_aero_beam_file(aero, grid, name, out_path)` | `blade.aerobeam` | 若提供 `AeroData`，则生成气动梁单元。|
| `gcs.py`        | `write_gcs_refs(rotors, out_path, add_blade_bases=True)` | `GCS.ref` | 写全局坐标系；通常由 `main.py` 调用一次。|

这些函数的输入输出遵循统一模式：
* 输入：结构/气动数据 + 网格对象 + 输出路径。
* 输出：无返回值，通过写文件产生副作用；出错时抛 `IOError` 或 `ValueError`。

---

## 机身与电机模块

### `gen_frameargu.py`

* `generate(aer_frameargu_xml: str, out_dir: str) -> Tuple[List[str], List[str]]`
  * 输入：`aer_frameargu.xml` 路径、输出目录。
  * 流程：
    1. 若 XML 不存在或缺 `<机身1>` 节点 → 返回空列表。
    2. 解析 `<原点位置>`, `<结构模型文件>`, `<模态结果文件>`。
    3. 只在 BDF/F06 文件都存在时：
       * 调 `femgen_mb.generate_fem(f06, bdf, frameargu.fem)` 生成 FEM。
       * 使用 `io_utils.write_text` 写 `frameargu.nod`（modal 结构节点，使用原点位置）与 `frameargu.elm`（唯一一行 include `frameargu.fem`）。
    4. 返回：
       * `node_includes = ['    include: "frameargu.nod";']`
       * `element_includes = ['    include: "frameargu.elm";']`
  * 副作用：在 `out_dir` 中创建 `frameargu.fem`, `frameargu.nod`, `frameargu.elm`；如生成 FEM 失败抛异常并终止（主程序可捕获）。

### `gen_motor.py`

* `generate(act_motor_xml: str, out_dir: str) -> Tuple[List[str], List[str]]`
  * 输入：`act_motor.xml` 路径、输出目录。
  * 流程：
    1. 若 XML 不存在 → 返回空列表。
    2. 解析 `<质量>`, `<质心位置>`, `<方向>`, `<转动惯量>`；缺失时默认 `0`。
    3. 写入 `motor.nod`（`structural: MOTOR, dynamic`）与 `motor.elm`（`body: MOTOR, MOTOR`），并返回对应 include 行。
  * 副作用：写两个文件；解析异常时会回退到默认值（除 `<质量>` 非数字抛错外）。

---

## 主装配写入器：`mbd_writer.py`

| 项 | 说明 |
|----|------|
| 职责 | 生成完整的 `main.mbd`，将所有节点/元素/控制参数按 MBDyn 语法写入。|
| 关键类型 | `RotorOut`, `SimParams`, `ControlVars`。|

### 主要接口

* `write_main_mbd(project_out_dir: str, rotor_outputs: List[RotorOut], sim: Optional[SimParams] = None, ctrl_override: Optional[Dict[str, int]] = None, c81_pairs: Optional[List[Tuple[str, str]]] = None, extra_includes_before_nodes: Optional[List[str]] = None, extra_elements_lines: Optional[List[str]] = None, theta_coll_deg: float = 5.0, include_aero: bool = True, *, extra_node_includes: Optional[List[str]] = None, extra_element_includes: Optional[List[str]] = None) -> str`
  * 输入：
    * `project_out_dir`：工程输出目录；所有 include 路径相对于该目录。
    * `rotor_outputs`：来自 `_run_single_blade` 的摘要，包含 `ref_path`, `nod_path`, `beam_path`, `body_path`, 可选 `aero_path`。
    * `sim`：仿真参数（时间步长、总时长等）；若未提供使用默认值。
    * `extra_node_includes` / `extra_element_includes`：由机身、电机等模块返回的 include 行（已缩进）。
    * 其他参数保持与旧版本兼容。
  * 输出：写入的 `main.mbd` 完整路径。
  * 副作用：
    1. 写 `begin: control data` → 控制变量数、自由度等（可由 `ctrl_override` 覆盖）。
    2. `begin: nodes;` 段：先写 `extra_node_includes`，再写旋翼节点、全局坐标系等。
    3. `begin: elements;` 段：写旋翼梁/气动文件 include，插入 `extra_element_includes`，再写控制器/输出等。
    4. 若 `include_aero` 为 `False`，跳过 `blade.aerobeam` include，并确保气动元素数量为 0。

---

## 其他模块

* `gcs.py`
  * `write_gcs_refs(rotors: Iterable[RotorCfg], out_path: str, add_blade_bases: bool = True) -> None`
  * 根据旋翼位置写 `GCS.ref`；通常在生成第一个桨叶后调用一次。
* `aero.py`
  * 详见桨叶写入表；主要依赖 `BladeGrid.interp_aero` 提供的气动分布。
* `tests/`
  * 包含单元测试与回归测试样例；运行方式使用 `pytest` 或自定义脚本（依项目而定）。

---

## 扩展建议

* 添加新组件（如传动轴、载荷）时，可仿照 `gen_motor.py` 编写新的 `gen_*.py`，并将其返回值拼接到 `extra_node_includes` / `extra_element_includes` 中。
* 若需要新的气动模式，可在 `aero.py` 中增加写入函数，并在 `_run_single_blade` 中插入生成逻辑。
* 更新模块或新增文件后，请同步修改本说明文档对应条目，以保持自动化协作的一致性。

