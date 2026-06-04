from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "pdptw_transfer_project_analysis.docx"


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=80, start=120, bottom=80, end=120):
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for m, v in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{m}"))
        if node is None:
            node = OxmlElement(f"w:{m}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(v))
        node.set(qn("w:type"), "dxa")


def set_table_width(table, widths):
    table.autofit = False
    for row in table.rows:
        for idx, width in enumerate(widths):
            row.cells[idx].width = width
            set_cell_margins(row.cells[idx])
            row.cells[idx].vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER


def set_repeat_table_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    tbl_header = OxmlElement("w:tblHeader")
    tbl_header.set(qn("w:val"), "true")
    tr_pr.append(tbl_header)


def add_table(doc, headers, rows, widths):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    hdr = table.rows[0]
    set_repeat_table_header(hdr)
    for i, h in enumerate(headers):
        cell = hdr.cells[i]
        cell.text = h
        set_cell_shading(cell, "F2F4F7")
        for p in cell.paragraphs:
            p.paragraph_format.space_after = Pt(0)
            p.runs[0].font.bold = True
            p.runs[0].font.size = Pt(9.5)
    for row_data in rows:
        row = table.add_row()
        for i, text in enumerate(row_data):
            cell = row.cells[i]
            cell.text = text
            for p in cell.paragraphs:
                p.paragraph_format.space_after = Pt(0)
                for run in p.runs:
                    run.font.size = Pt(9)
    set_table_width(table, widths)
    doc.add_paragraph()
    return table


def add_callout(doc, title, body):
    table = doc.add_table(rows=1, cols=1)
    table.style = "Table Grid"
    cell = table.cell(0, 0)
    set_cell_shading(cell, "F4F6F9")
    set_cell_margins(cell, top=140, start=180, bottom=140, end=180)
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(3)
    r = p.add_run(title)
    r.bold = True
    r.font.color.rgb = RGBColor(31, 77, 120)
    r.font.size = Pt(10.5)
    p2 = cell.add_paragraph()
    p2.paragraph_format.space_after = Pt(0)
    p2.paragraph_format.line_spacing = 1.1
    for part in body.split("\n"):
        if p2.runs:
            p2.add_run().add_break()
        p2.add_run(part)
    doc.add_paragraph()


def add_kv_table(doc, rows):
    add_table(
        doc,
        ["项目", "说明"],
        rows,
        [Inches(1.55), Inches(4.8)],
    )


def add_bullet(doc, text):
    p = doc.add_paragraph(style="List Bullet")
    p.paragraph_format.left_indent = Inches(0.5)
    p.paragraph_format.first_line_indent = Inches(-0.25)
    p.paragraph_format.space_after = Pt(4)
    p.add_run(text)


def add_number(doc, text):
    p = doc.add_paragraph(style="List Number")
    p.paragraph_format.left_indent = Inches(0.5)
    p.paragraph_format.first_line_indent = Inches(-0.25)
    p.paragraph_format.space_after = Pt(4)
    p.add_run(text)


def add_code_block(doc, lines):
    table = doc.add_table(rows=1, cols=1)
    table.style = "Table Grid"
    cell = table.cell(0, 0)
    set_cell_shading(cell, "F7F7F7")
    set_cell_margins(cell, top=120, start=160, bottom=120, end=160)
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    for idx, line in enumerate(lines):
        if idx:
            p.add_run().add_break()
        run = p.add_run(line)
        run.font.name = "Courier New"
        run._element.rPr.rFonts.set(qn("w:eastAsia"), "Courier New")
        run.font.size = Pt(9)
    doc.add_paragraph()


def style_document(doc):
    section = doc.sections[0]
    section.page_width = Inches(8.5)
    section.page_height = Inches(11)
    section.top_margin = Inches(1)
    section.bottom_margin = Inches(1)
    section.left_margin = Inches(1)
    section.right_margin = Inches(1)
    section.header_distance = Inches(0.492)
    section.footer_distance = Inches(0.492)

    styles = doc.styles
    normal = styles["Normal"]
    normal.font.name = "Calibri"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    normal.font.size = Pt(11)
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.1

    title = styles["Title"]
    title.font.name = "Calibri Light"
    title._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    title.font.size = Pt(24)
    title.font.color.rgb = RGBColor(11, 37, 69)
    title.paragraph_format.space_after = Pt(8)

    subtitle = styles["Subtitle"]
    subtitle.font.name = "Calibri"
    subtitle._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    subtitle.font.size = Pt(11)
    subtitle.font.color.rgb = RGBColor(85, 85, 85)
    subtitle.paragraph_format.space_after = Pt(18)

    for style_name, size, color, before, after in [
        ("Heading 1", 16, "2E74B5", 16, 8),
        ("Heading 2", 13, "2E74B5", 12, 6),
        ("Heading 3", 12, "1F4D78", 8, 4),
    ]:
        style = styles[style_name]
        style.font.name = "Calibri"
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        style.font.size = Pt(size)
        style.font.color.rgb = RGBColor.from_string(color)
        style.font.bold = True
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)


def build():
    doc = Document()
    style_document(doc)

    title = doc.add_paragraph(style="Title")
    title.add_run("pdptw_transfer 项目解读报告")
    subtitle = doc.add_paragraph(style="Subtitle")
    subtitle.add_run("面向“带时间窗取送货与一次转运”的两阶段 Gurobi 优化原型分析")

    add_callout(
        doc,
        "一句话概览",
        "本项目围绕快递网点间的批量运输调度问题建模：在车辆容量、时间窗、装卸服务时间和转运处理时间约束下，选择直达或一次转运方案，并尝试将运输弧进一步排成车辆执行序列，以最小化固定车辆成本、行驶成本和转运操作成本。",
    )

    doc.add_heading("1. 项目定位", level=1)
    doc.add_paragraph(
        "代码实现的是一个 PDPTW with Transfer（带时间窗取送货问题，允许一次转运）的求解原型。"
        "从文件组织和模型结构看，它并不是一个完整产品化调度系统，而是一个数学建模与实验求解项目："
        "数据以 CSV 给出，模型由 Gurobi 构建，主程序固定读取某一规模算例并运行求解。"
    )
    add_kv_table(
        doc,
        [
            ("业务问题", "网点之间存在快递批次需求，每批快递有起点、终点、最早装车时间、最晚卸车时间和票数。"),
            ("关键决策", "每批需求选择直达运输，或选择一个可转运网点进行一次中转。"),
            ("资源假设", "车辆同车型、数量不限、有容量上限、固定使用成本；车辆没有固定始发场站或终点场站。"),
            ("优化目标", "最小化车辆固定成本、车辆行驶时间成本和快递转运操作成本之和。"),
            ("当前实现状态", "第一阶段模型已实现并在入口中求解；第二阶段调度模型已构造但主程序未实际 optimize。"),
        ],
    )

    doc.add_heading("2. 文件结构与职责", level=1)
    add_table(
        doc,
        ["路径", "职责", "解读"],
        [
            ("src/solve.py", "主入口", "固定 num_of_nodes=50，读取数据，构造 Instance，求解第一阶段，再生成第二阶段输入。"),
            ("src/Problem.py", "问题实例", "定义 Request 和 Instance，汇总节点、转运点、需求、时间矩阵、容量、服务时间和成本参数。"),
            ("src/model_of_first_stage.py", "第一阶段模型", "选择运输弧 x[i,j] 与转运决策 y[o,t,d]，并加入时间窗、容量和成本约束。"),
            ("src/model_of_second_stage.py", "第二阶段模型", "把第一阶段选出的运输弧视作任务，尝试安排任务之间的衔接顺序。"),
            ("src/uilts.py", "工具函数", "读取 CSV，并将第一阶段解转换为车辆任务、快递流和转运前后任务配对。"),
            ("data/*.csv", "算例数据", "包含 30、40、50 节点规模的节点表、需求表、时间矩阵，以及共用参数表。"),
            ("src/说明文档.pdf", "原始题意说明", "描述问题背景、输入字段、成本计算逻辑、约束和提交要求。"),
        ],
        [Inches(1.65), Inches(1.35), Inches(3.35)],
    )

    doc.add_heading("3. 数据与参数", level=1)
    doc.add_paragraph("项目包含三组算例，规模随节点数增长；需求数量近似覆盖所有有向 OD 组合，因此 50 节点算例已经具有较大的建模规模。")
    add_table(
        doc,
        ["算例", "节点数", "转运点数", "需求批次数", "需求总票数", "单批最大票数"],
        [
            ("30 节点", "30", "8", "455", "10,904", "50"),
            ("40 节点", "40", "10", "816", "19,710", "52"),
            ("50 节点", "50", "11", "1,275", "30,170", "61"),
        ],
        [Inches(0.9), Inches(0.78), Inches(0.9), Inches(1.05), Inches(1.1), Inches(1.25)],
    )
    add_table(
        doc,
        ["参数", "当前值", "含义"],
        [
            ("vehicle_capacity", "100", "车辆装载容量上限。"),
            ("vehicle_service_time", "100", "车辆在一个网点的装卸货服务时长。"),
            ("parcel_transfer_time", "400", "快递在转运网点完成转运处理所需时间。"),
            ("vehicle_fixed_cost", "2000", "启用一辆车或一条任务弧时计入的固定成本。"),
            ("vehicle_unit_travel_cost", "1", "单位行驶时间成本，当前代码隐含为 1。"),
            ("parcel_transfer_unit_cost", "1", "单位票数转运成本，当前代码隐含为 1。"),
        ],
        [Inches(1.75), Inches(0.9), Inches(3.7)],
    )

    doc.add_heading("4. 运行方式与依赖", level=1)
    doc.add_paragraph(
        "项目没有 requirements.txt 或 pyproject.toml。实际运行依赖 pandas 和 gurobipy。"
        "当前 solve.py 使用 '../data/...' 相对路径，因此预期从 src 目录启动。"
    )
    add_code_block(doc, ["cd /Users/didi/PycharmProjects/pdptw_transfer/src", "python3 solve.py"])
    add_callout(
        doc,
        "运行注意",
        "从项目根目录直接执行 python3 src/solve.py 时，数据路径会解析到项目外层的 ../data，容易触发 FileNotFoundError。建议将入口路径改为基于 __file__ 的项目根目录定位。",
    )

    doc.add_heading("5. 总体求解流程", level=1)
    doc.add_paragraph("当前实现采用两阶段思路。第一阶段处理“需求经哪条弧运输”，第二阶段处理“这些运输弧如何被车辆按时间顺序执行”。")
    add_table(
        doc,
        ["阶段", "输入", "核心输出", "作用"],
        [
            ("第一阶段", "Instance：节点、转运点、需求、时间矩阵、参数", "x：开通运输弧；y：需求转运选择；a/d：弧起止时间变量", "决定每批快递直达或一次转运，并控制弧容量与基础时间可行性。"),
            ("解转换", "第一阶段中 x/y 的取值", "vehicle_parcel、vehicles、arcs、pairs", "把开通弧编号为车辆任务，并识别转运包裹对应的前后任务关系。"),
            ("第二阶段", "每条任务的时间窗、任务对、弧端点", "任务衔接变量 x[i,j] 与任务到发时间", "尝试把任务串联成车辆可执行的排程，进一步估计车辆数和衔接成本。"),
        ],
        [Inches(1.0), Inches(1.85), Inches(1.75), Inches(1.75)],
    )

    doc.add_heading("6. 第一阶段模型解读", level=1)
    doc.add_paragraph("第一阶段位于 src/model_of_first_stage.py，是当前项目最完整的核心模型。")
    add_table(
        doc,
        ["符号/变量", "代码位置", "含义"],
        [
            ("x[i,j]", "self.x", "二元变量，表示节点 i 到节点 j 的运输弧是否开通。"),
            ("y[o,t,d]", "self.y", "二元变量，表示从 o 到 d 的需求是否选择在 t 转运。"),
            ("a[i,j]", "self.a", "运输弧 i->j 的出发/开始服务相关时间变量。"),
            ("d[i,j]", "self.d", "运输弧 i->j 的到达/结束相关时间变量。"),
            ("travel_cost", "目标辅助变量", "所有开通弧的行驶时间成本上界。"),
            ("transfer_cost", "目标辅助变量", "发生转运的快递票数成本上界。"),
            ("fixed_vehicle_cost", "目标辅助变量", "开通弧数量乘车辆固定成本的上界。"),
        ],
        [Inches(1.25), Inches(1.35), Inches(3.75)],
    )
    doc.add_heading("6.1 路径选择约束", level=2)
    doc.add_paragraph(
        "对每个请求 (o,d)，模型要求：如果直达弧 x[o,d] 被选中，则不选择转运；如果不直达，则必须在可转运点集合 T 中选择且只选择一个转运点。"
    )
    add_code_block(doc, ["sum(y[o,t,d] for t in T if t != o and t != d) == 1 - x[o,d]"])
    doc.add_heading("6.2 转运联动约束", level=2)
    doc.add_paragraph(
        "如果请求选择 o -> t -> d，则模型强制开通 o -> t 和 t -> d 两条运输弧。这保证了转运决策不会悬空。"
    )
    add_code_block(doc, ["x[o,t] >= y[o,t,d]", "x[t,d] >= y[o,t,d]"])
    doc.add_heading("6.3 时间窗与转运处理约束", level=2)
    doc.add_paragraph(
        "模型使用 Big-M 方式，根据 x 或 y 是否为 1 来启用时间窗约束：始发端需要晚于最早装车时间，目的端需要早于最晚卸车时间，转运点需要满足处理时长。"
    )
    add_bullet(doc, "直达：r.e + service_time <= a[o,d]，且 d[o,d] + service_time <= r.l。")
    add_bullet(doc, "转运：第一段满足始发时间窗，第二段满足目的时间窗；两段之间满足 parcel_transfer_time。")
    add_bullet(doc, "自然行驶时间：a[i,j] + travel_time[i,j] <= d[i,j]。")
    doc.add_heading("6.4 容量约束", level=2)
    doc.add_paragraph(
        "容量约束按弧聚合快递票数：进入转运点、离开转运点、转运点之间的弧分别建约束，确保弧上承载的需求总票数不超过车辆容量 Q。"
    )

    doc.add_heading("7. 第二阶段模型解读", level=1)
    doc.add_paragraph(
        "第二阶段位于 src/model_of_second_stage.py。它接收第一阶段选出的运输弧，并把每条弧视作一个待执行任务。"
        "模型目标包含从虚拟节点 0 出发的固定成本、任务本身的行驶成本，以及任务之间空驶衔接成本。"
    )
    add_table(
        doc,
        ["元素", "含义"],
        [
            ("keys", "所有第一阶段任务编号，加上虚拟节点 0。"),
            ("x[i,j]", "任务 i 后接任务 j 的二元变量。"),
            ("a[i], d[i]", "任务 i 的开始和结束时间。"),
            ("pairs", "同一转运快递的前段任务和后段任务，要求前段完成后才能进行后段。"),
            ("vehicles[i]", "任务 i 对应的实际运输弧，例如 (origin, destination)。"),
        ],
        [Inches(1.45), Inches(4.9)],
    )
    add_callout(
        doc,
        "当前关键状态",
        "主程序已经构造 second_stage_model，但 optimize 被注释掉。因此当前仓库实际执行时只会输出第一阶段成本，不会给出第二阶段车辆排程结果。",
    )

    doc.add_heading("8. 已发现的主要风险与改进点", level=1)
    add_table(
        doc,
        ["优先级", "问题", "影响", "建议"],
        [
            ("高", "第二阶段未求解", "程序不会输出完整车辆调度方案。", "取消注释 optimize，并补充状态判断、结果打印和不可行诊断。"),
            ("高", "第二阶段缺少完整路径结构", "仅一进一出可能产生子环或不符合真实车辆连续执行逻辑的任务环。", "加入 depot 逻辑、子环消除或时间递推约束，并明确“车辆数”的定义。"),
            ("中", "入口路径依赖运行目录", "从项目根目录运行容易找不到数据。", "用 Path(__file__).resolve() 定位项目根目录。"),
            ("中", "第一阶段 y 对所有节点创建", "非转运点 y 变量冗余，扩大模型规模。", "只对 t in T 创建 y[o,t,d]。"),
            ("中", "未使用弧仍受自然时间约束", "a[i,j] + c[i,j] <= d[i,j] 对 x=0 也生效，可能带来无意义限制。", "改为 Big-M 条件约束：a + c <= d + M(1-x)。"),
            ("中", "目标函数未使用参数表中的单位成本", "当前 travel/transfer 单位成本被隐含为 1，参数字段未真正参与。", "在 Instance 中读取并使用 vehicle_unit_travel_cost 与 parcel_transfer_unit_cost。"),
            ("低", "变量名重复或不够可读", "Gurobi 日志和 LP 文件不便调试。", "第二阶段变量命名改为 x_i_j。"),
            ("低", "生成文件未忽略", "model.lp、model.ilp、__pycache__ 可能污染仓库。", "增加 .gitignore。"),
        ],
        [Inches(0.72), Inches(1.45), Inches(2.0), Inches(2.18)],
    )

    doc.add_heading("9. 推荐整理路线", level=1)
    add_number(doc, "先做工程清理：添加 .gitignore，修复数据路径，增加命令行参数 num_of_nodes。")
    add_number(doc, "跑通小规模闭环：用 30 节点算例先完整求解两阶段，并输出 x/y、车辆任务、总成本分解。")
    add_number(doc, "修正第一阶段冗余变量与条件时间约束，减少无效模型规模和潜在不可行风险。")
    add_number(doc, "完善第二阶段调度模型，明确车辆从虚拟源点出发、执行若干任务、回到虚拟终点或结束的路径结构。")
    add_number(doc, "补充实验设计：比较无转运、允许转运、不同容量、不同固定成本、不同时间窗宽度下的成本和车辆数变化。")
    add_number(doc, "增加结果可视化：展示转运点使用频次、被开通的运输弧、车辆任务甘特图。")

    doc.add_heading("10. 结论", level=1)
    doc.add_paragraph(
        "这个项目已经具备清晰的题意输入、第一阶段建模框架和两阶段求解思路，适合作为算法实验或课程建模项目继续推进。"
        "目前最需要补强的是第二阶段调度模型的完整性和程序运行闭环。"
        "如果目标是形成可提交或可复现实验报告，建议优先保证 30 节点实例两阶段稳定运行，再逐步扩展到 40 和 50 节点，并记录求解时间、目标值、车辆数、转运票数等指标。"
    )

    footer = doc.sections[0].footer.paragraphs[0]
    footer.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    run = footer.add_run("pdptw_transfer 项目解读报告")
    run.font.size = Pt(8)
    run.font.color.rgb = RGBColor(85, 85, 85)

    doc.save(OUT)


if __name__ == "__main__":
    build()
