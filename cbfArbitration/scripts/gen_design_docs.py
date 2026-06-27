#!/Users/qingxu/.ai-env/bin/python3
"""
批量生成 8 个核心函数的 LaTeX 设计文档（符合 03_design_doc_gen.md 模板）
输出路径：doc/<FunctionName>/<FunctionName>.tex
"""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOC = ROOT / "doc"

AUTHOR = "许庆"
CONTACT = "18612426814 / qingxu@tsinghua.edu.cn"
SECRECY = "内部公开"
AI_MODEL = "Claude-Opus-4.7"
TODAY = "2026 年 6 月 26 日"

FUNCTIONS = [
    {
        "name": "rotateVectorByMinusYaw",
        "cn": "全局向量绕 -yaw 旋转到自车系",
        "category": "element",
        "purpose": "将全局坐标系下的二维向量左乘旋转矩阵 R(-yaw)，输出 ego frame 下分量",
        "inputs": [
            ("vxGlobal", "double", "全局 x 分量 (m/s 或 m/s^2)"),
            ("vyGlobal", "double", "全局 y 分量"),
            ("yaw", "double", "自车朝向角 (rad)"),
        ],
        "outputs": [
            ("vxEgo", "double*", "ego frame 纵向分量"),
            ("vyEgo", "double*", "ego frame 横向分量"),
        ],
        "formula": r"\begin{bmatrix} v_x^{ego} \\ v_y^{ego} \end{bmatrix} = \begin{bmatrix} \cos\varphi & \sin\varphi \\ -\sin\varphi & \cos\varphi \end{bmatrix}\begin{bmatrix} v_x^{global} \\ v_y^{global} \end{bmatrix}",
        "steps": [
            ("calc_cos_sin", "计算 cos(yaw)、sin(yaw)"),
            ("apply_rotation", "应用 R(-yaw) 得到 ego frame 分量"),
            ("write_back", "通过指针写回输出"),
        ],
        "children": [],
    },
    {
        "name": "computeBarrierSquared",
        "cn": "平方型 Barrier 函数计算",
        "category": "element",
        "purpose": "在 ego frame 下计算 B = dx^2 + dy^2 - r_safe^2",
        "inputs": [
            ("dxEgo", "double", "纵向相对位置 (m)"),
            ("dyEgo", "double", "横向相对位置 (m)"),
            ("safetyRadius", "double", "安全圆半径 r_safe (m)"),
        ],
        "outputs": [("return", "double", "Barrier 值 B")],
        "formula": r"B(x_1) = dx^2 + dy^2 - r_{safe}^2",
        "steps": [
            ("dist_sq", "distSquared = dx*dx + dy*dy"),
            ("radius_sq", "radiusSquared = r_safe * r_safe"),
            ("diff", "B = distSquared - radiusSquared"),
        ],
        "children": [],
    },
    {
        "name": "computeBarrierFirstDerivative",
        "cn": "Barrier 一阶导数计算",
        "category": "element",
        "purpose": "求平方型 Barrier 函数沿时间的一阶导数 dotB",
        "inputs": [
            ("dxEgo", "double", "纵向相对位置"),
            ("dyEgo", "double", "横向相对位置"),
            ("vRxEgo", "double", "纵向相对速度"),
            ("vRyEgo", "double", "横向相对速度"),
        ],
        "outputs": [("return", "double", "一阶导数 dotB")],
        "formula": r"\dot B = 2(dx \cdot v_{rx} + dy \cdot v_{ry})",
        "steps": [
            ("dot_prod", "innerProduct = dx*vRx + dy*vRy"),
            ("scale", "dotB = 2 * innerProduct"),
        ],
        "children": [],
    },
    {
        "name": "computeBarrierSecondDerivativeFree",
        "cn": "Barrier 二阶导数自由项",
        "category": "element",
        "purpose": "求 Barrier 二阶导数中与控制 u 无关的部分（用于 HOCBF 仿射约束装配）",
        "inputs": [
            ("dxEgo", "double", "纵向相对位置"),
            ("dyEgo", "double", "横向相对位置"),
            ("vRxEgo", "double", "纵向相对速度"),
            ("vRyEgo", "double", "横向相对速度"),
            ("axObEgo", "double", "障碍物绝对加速度 ego-x 分量"),
            ("ayObEgo", "double", "障碍物绝对加速度 ego-y 分量"),
        ],
        "outputs": [("return", "double", "二阶导数自由项 ddBFree")],
        "formula": r"\ddot B|_{free} = 2(v_{rx}^2 + v_{ry}^2) + 2(dx \cdot a_x^{ob} + dy \cdot a_y^{ob})",
        "steps": [
            ("vel_sq", "velSq = vRx^2 + vRy^2"),
            ("ob_acc", "obAccTerm = dx*ax + dy*ay"),
            ("sum", "ddBFree = 2*velSq + 2*obAccTerm"),
        ],
        "children": [],
    },
    {
        "name": "computeObstacleAffineConstraint",
        "cn": "单障碍物 HOCBF 仿射约束生成",
        "category": "component",
        "purpose": "整合 B、dotB、ddBFree，生成形如 betaA*a + betaT*tan(δ) >= bound 的仿射约束",
        "inputs": [
            ("egoVelocity", "double", "自车速度 v"),
            ("wheelBase", "double", "轴距 L"),
            ("alpha1", "double", "HOCBF 一阶系数 α1"),
            ("alpha2", "double", "HOCBF 二阶系数 α2"),
            ("obstacle", "ObstacleState", "障碍物 ego-frame 状态"),
            ("axObEgo", "double", "障碍物 ego-frame 加速度 x"),
            ("ayObEgo", "double", "障碍物 ego-frame 加速度 y"),
            ("safetyRadius", "double", "安全半径"),
        ],
        "outputs": [("return", "AffineConstraint", "{betaA, betaT, bound}")],
        "formula": r"\beta_a a + \beta_t \tan\delta_f \geq -\bigl(\ddot B|_{free} + (\alpha_1+\alpha_2)\dot B + \alpha_1\alpha_2 B\bigr)",
        "steps": [
            ("call_b", "调用 computeBarrierSquared 求 B"),
            ("call_dotb", "调用 computeBarrierFirstDerivative 求 dotB"),
            ("call_ddb", "调用 computeBarrierSecondDerivativeFree 求 ddBFree"),
            ("coef", "betaA=-2dx, betaT=-2 v^2/L * dy"),
            ("bound", "bound = -(ddBFree+(α1+α2)dotB+α1α2 B)"),
        ],
        "children": [
            ("computeBarrierSquared", "B 值"),
            ("computeBarrierFirstDerivative", "dotB"),
            ("computeBarrierSecondDerivativeFree", "ddBFree"),
        ],
    },
    {
        "name": "clampControlInput",
        "cn": "控制输入双轴饱和限幅",
        "category": "element",
        "purpose": "对 (a, δ_f) 做 box 限幅，确保不越界",
        "inputs": [
            ("aRaw", "double", "限幅前加速度"),
            ("deltaFRaw", "double", "限幅前前轮转角"),
            ("aMin/aMax", "double", "加速度上下界"),
            ("deltaFMin/deltaFMax", "double", "转角上下界"),
        ],
        "outputs": [
            ("aFinal", "double*", "限幅后加速度"),
            ("deltaFFinal", "double*", "限幅后转角"),
        ],
        "formula": r"a^* = \max(a_{min}, \min(a_{max}, a)),\quad \delta^*_f = \max(\delta_{min}, \min(\delta_{max}, \delta_f))",
        "steps": [
            ("clamp_a", "对加速度 std::clamp"),
            ("clamp_delta", "对转角 std::clamp"),
        ],
        "children": [],
    },
    {
        "name": "solveQpHildreth",
        "cn": "Hildreth 对偶坐标下降求解二维 QP",
        "category": "element",
        "purpose": "求解 (1/2)u^T Q u + c^T u s.t. Au<=b 的二维 QP",
        "inputs": [
            ("qDiagA, qDiagT", "double", "Q 对角元素"),
            ("c", "double[2]", "线性项"),
            ("aMatRowMajor", "double*", "约束矩阵 A（行优先）"),
            ("bVec", "double*", "约束右端 b"),
            ("rowCount", "size_t", "实际约束行数"),
            ("maxIter", "size_t", "最大迭代步"),
            ("tolerance", "double", "收敛阈值"),
        ],
        "outputs": [
            ("uSolution", "double[2]", "QP 最优解"),
            ("iterUsed", "size_t*", "实际迭代步"),
            ("return", "bool", "是否可行"),
        ],
        "formula": r"u^* = \arg\min_{u} \tfrac{1}{2}u^\top Q u + c^\top u\quad s.t.\ Au\le b",
        "steps": [
            ("init_uncons", "无约束最优 u = -Q^-1 c"),
            ("check_feas", "若所有 Au<=b 已成立则直接返回"),
            ("hildreth_iter", "对偶变量逐分量更新 λ_i = max(0, λ_i + (b_i - A_i u)/H_ii)"),
            ("primal_recover", "u = -Q^-1 (c + A^T λ)"),
        ],
        "children": [],
    },
    {
        "name": "cbfControlRevision",
        "cn": "CBF 控制修正模块顶层入口",
        "category": "component",
        "purpose": "装配多障碍 HOCBF 仿射约束 + 输入 box，调用 QP 求解器，输出安全控制",
        "inputs": [
            ("egoState", "EgoState", "自车状态 + 原始控制"),
            ("obstacles", "ObstacleState[]", "障碍物数组"),
            ("obstacleNum", "size_t", "障碍物数量"),
            ("param", "CbfParam", "算法参数"),
        ],
        "outputs": [("return", "CbfOutput", "{aSafe, deltaFSafe, feasible, ...}")],
        "formula": r"u^* = \arg\min_{u\in\mathcal U}(u-u_o)^\top Q(u-u_o)\ s.t.\ m_{1,i}(u)\geq 0,\ \forall i",
        "steps": [
            ("rotate_acc", "障碍物全局加速度旋转到 ego frame"),
            ("gen_cons", "逐障碍物调用 computeObstacleAffineConstraint"),
            ("box_cons", "装配 4 条 box 约束"),
            ("call_qp", "调用 solveQpHildreth"),
            ("post_clamp", "atan(tanδ) + clampControlInput"),
        ],
        "children": [
            ("rotateVectorByMinusYaw", "ego-frame 加速度"),
            ("computeObstacleAffineConstraint", "单条 HOCBF 约束"),
            ("solveQpHildreth", "QP 求解"),
            ("clampControlInput", "终值限幅"),
        ],
    },
]

def tex_escape(s: str) -> str:
    """对纯文本（非数学模式）做 LaTeX 转义。"""
    greek_map = {
        "α": r"$\alpha$",
        "β": r"$\beta$",
        "γ": r"$\gamma$",
        "δ": r"$\delta$",
        "Δ": r"$\Delta$",
        "λ": r"$\lambda$",
        "φ": r"$\varphi$",
        "θ": r"$\theta$",
        "≥": r"$\geq$",
        "≤": r"$\leq$",
        "→": r"$\rightarrow$",
        "≠": r"$\neq$",
    }
    for k, v in greek_map.items():
        s = s.replace(k, v)
    out = []
    for ch in s:
        if ch == "\\":
            out.append(r"\textbackslash{}")
        elif ch in "&%#":
            out.append("\\" + ch)
        elif ch == "_":
            out.append(r"\_")
        elif ch == "^":
            out.append(r"\textasciicircum{}")
        elif ch == "~":
            out.append(r"\textasciitilde{}")
        else:
            out.append(ch)
    return "".join(out)


PREAMBLE = r"""\documentclass[a4paper,10pt]{article}
\usepackage[UTF8]{ctex}
\usepackage{geometry}
\usepackage{tikz}
\usetikzlibrary{shapes.geometric, arrows.meta, positioning, calc, trees}
\usepackage{longtable}
\usepackage{array}
\usepackage{fancyhdr}
\usepackage{xcolor}
\usepackage{amsmath}
\usepackage{amssymb}
\usepackage{hyperref}
\geometry{left=2.5cm, right=2.5cm, top=3cm, bottom=2.5cm}
\pagestyle{fancy}
\fancyhf{}
\fancyhead[C]{\small %s 函数设计文档}
\fancyfoot[C]{\thepage}
\setcounter{secnumdepth}{3}
\setcounter{tocdepth}{3}
\tikzset{
    startstop/.style={ellipse, draw, fill=red!10, text width=8em, minimum height=0.9cm, align=center},
    process/.style={rectangle, draw, fill=blue!5, text width=11em, minimum height=1.1cm, align=center},
    decision/.style={diamond, draw, fill=green!10, text width=7em, minimum height=1cm, align=center},
    arrow/.style={thick,->,>=Stealth}
}
"""

def cover(name, cn):
    return r"""\begin{titlepage}
    \centering
    {\Huge\textbf{%s（%s）函数设计}\par}
    \vspace{1.5cm}
    \renewcommand{\arraystretch}{1.5}
    \begin{tabular}{|l|l|l|l|}
        \hline
        文件密级 & %s & 编号 & CBF-DOC-%s \\ \hline
        版本 & V1.0 & 内容 & %s（%s）函数设计 \\ \hline
        设计 & %s & 联系方式 & %s \\ \hline
        审核 & - & 审批 & - \\ \hline
        发布日期 & %s & & \\ \hline
    \end{tabular}
    \vspace{1.5cm}
    \section*{文件更改记录}
    \begin{longtable}{|c|c|p{4cm}|c|c|c|c|}
        \hline
        日期 & 版本号 & 修订说明 & 辅助AI & 修订 & 审核 & 审批 \\ \hline
        2026-6-26 & V1.0 & 初次设计 & %s & %s & - & - \\ \hline
    \end{longtable}
\end{titlepage}
\tableofcontents
\newpage
""" % (name, cn, SECRECY, name.upper(), name, cn, AUTHOR, CONTACT, TODAY, AI_MODEL, AUTHOR)


def section_requirements(f):
    return r"""\section{公有函数设计}
\subsection{%s（%s）函数设计}
\subsubsection{函数需求}
\par 本函数实现 \textbf{%s}。其在 CBF 自动驾驶安全控制流水线中承担如下职责：%s。
\par 头文件路径：\texttt{include/cpp/%s/%s.hpp}；源文件路径：\texttt{src/cpp/%s/%s.cpp}。
""" % (tex_escape(f["name"]), f["cn"], f["cn"], tex_escape(f["purpose"]),
       tex_escape(f["name"]), tex_escape(f["name"]),
       tex_escape(f["name"]), tex_escape(f["name"]))


def section_algorithm(f):
    s = r"""\subsubsection{算法设计}
\par 本函数核心数学表达式如下：
\begin{equation}
%s
\end{equation}
\par 实现步骤：
\begin{enumerate}
""" % f["formula"]
    for sid, desc in f["steps"]:
        s += r"  \item \textbf{%s}: %s" % (tex_escape(sid), tex_escape(desc)) + "\n"
    s += "\\end{enumerate}\n"
    return s


def section_flowchart(f):
    nodes = ""
    arrows = ""
    nodes += "    \\node (start) [startstop] {开始};\n"
    prev = "start"
    for i, (sid, desc) in enumerate(f["steps"]):
        nid = "p%d" % i
        clean = tex_escape(desc)
        nodes += "    \\node (%s) [process, below of=%s, yshift=-0.3cm] {%s};\n" % (nid, prev, clean)
        arrows += "    \\draw [arrow] (%s) -- (%s);\n" % (prev, nid)
        prev = nid
    nodes += "    \\node (end) [startstop, below of=%s, yshift=-0.3cm] {返回 / 写回输出};\n" % prev
    arrows += "    \\draw [arrow] (%s) -- (end);\n" % prev
    return r"""\subsubsection{程序流程图}
\begin{figure}[h]
    \centering
    \begin{tikzpicture}[node distance=1.6cm]
%s%s
    \end{tikzpicture}
    \caption{%s 函数程序流程图}
    \label{fig:flow_%s}
\end{figure}
""" % (nodes, arrows, tex_escape(f["name"]), f["name"])


def section_io(f):
    in_items = "\n".join([
        "  \\item \\texttt{%s} (%s) -- %s" % (tex_escape(n), tex_escape(t), tex_escape(d))
        for n, t, d in f["inputs"]
    ])
    out_items = "\n".join([
        "  \\item \\texttt{%s} (%s) -- %s" % (tex_escape(n), tex_escape(t), tex_escape(d))
        for n, t, d in f["outputs"]
    ])
    return r"""\subsubsection{函数接口}
\par 输入参数列表：
\begin{itemize}
%s
\end{itemize}
\par 输出参数列表：
\begin{itemize}
%s
\end{itemize}
\subsubsection{输入输出模块图}
\begin{figure}[h]
    \centering
    \begin{tikzpicture}[>=Stealth]
        \node[draw, rectangle, minimum width=4.5cm, minimum height=2cm, fill=blue!5] (func) at (0,0) {\large{\textbf{%s}}};
        \draw[->, thick] (-3.5,0.4) -- node[above]{I (输入)} (-2.3,0.4);
        \draw[->, thick] (2.3,0.4) -- node[above]{O (输出)} (3.5,0.4);
        \draw[->, thick] (0,-1.0) -- node[right]{f (返回)} (0,-2.0);
    \end{tikzpicture}
    \caption{%s 函数输入输出模块图}
    \label{fig:io_%s}
\end{figure}
""" % (in_items, out_items, tex_escape(f["name"]), tex_escape(f["name"]), f["name"])


def section_metadata(f):
    cat = "元件" if f["category"] == "element" else "组件"
    return r"""\subsubsection{函数诸元表}
\begin{table}[h]
    \centering
    \caption{%s 函数诸元表}
    \label{tab:meta_%s}
    \begin{tabular}{|c|c|c|c|c|}
        \hline
        中文名 & 英文名 & 性质 & 级别 & 入库 \\ \hline
        %s & %s & %s & 应用层 & 是 \\ \hline
    \end{tabular}
\end{table}
""" % (tex_escape(f["name"]), f["name"], f["cn"], tex_escape(f["name"]), cat)


def section_arch(f):
    if not f["children"]:
        body = r"""\par 本函数为\textbf{元件}（叶子节点），无下级函数依赖。
"""
    else:
        kids_nodes = ""
        for j, (n, _) in enumerate(f["children"]):
            kids_nodes += "        child {node[draw, fill=yellow!10] {%s}}\n" % tex_escape(n)
        body = r"""\begin{figure}[h]
    \centering
    \begin{tikzpicture}[>=Stealth, level distance=2cm,
        level 1/.style={sibling distance=4cm}]
        \node[draw, fill=blue!10] {%s}
%s        ;
    \end{tikzpicture}
    \caption{%s 函数架构图}
    \label{fig:arch_%s}
\end{figure}
\subsubsection{下级函数需求}
\begin{itemize}
%s
\end{itemize}
""" % (
            tex_escape(f["name"]),
            kids_nodes,
            tex_escape(f["name"]), f["name"],
            "\n".join(["  \\item \\texttt{%s}：%s" % (tex_escape(n), tex_escape(d)) for n, d in f["children"]])
        )
    return r"""\subsubsection{函数架构}
%s
""" % body


def section_test(f):
    if f["name"] == "computeBarrierSquared":
        normal = [
            ("TC1", "正前方 10m", "dx=10,dy=0,r=4", "B=84"),
            ("TC2", "斜向 5m", "dx=3,dy=4,r=4", "B=9"),
            ("TC3", "重合", "dx=0,dy=0,r=4", "B=-16 (碰撞)"),
            ("TC4", "正左 5m", "dx=0,dy=5,r=2", "B=21"),
            ("TC5", "对角线", "dx=2,dy=2,r=2", "B=4"),
        ]
        abnormal = [
            ("EC1", "NaN 输入", "dx=NaN", "返回 NaN"),
            ("EC2", "Inf 输入", "dx=Inf", "返回 Inf"),
            ("EC3", "负 r", "r=-3", "B 仍可计算（数值定义合法）"),
            ("EC4", "极大值", "dx=1e308", "可能溢出 Inf"),
        ]
    else:
        normal = [("TC%d" % (i + 1), "正常用例 %d" % (i + 1), "见 cases.json", "见 cases.json") for i in range(5)]
        abnormal = [
            ("EC1", "NaN 输入", "任一标量=NaN", "输出含 NaN"),
            ("EC2", "Inf 输入", "任一标量=Inf", "输出含 Inf"),
            ("EC3", "数学边界", "极限值", "结果有界或返回 false"),
            ("EC4", "QP 不可行", "约束相互冲突", "返回 feasible=false"),
        ]
    n_rows = "\n".join(["  %s & %s & %s & %s \\\\ \\hline" % tuple(tex_escape(x) for x in t) for t in normal])
    a_rows = "\n".join(["  %s & %s & %s & %s \\\\ \\hline" % tuple(tex_escape(x) for x in t) for t in abnormal])
    return r"""\subsubsection{测试用例表}
\noindent\textbf{正常场景测试用例如表~\ref{tab:normal_%s} 所示：}
\begin{table}[h]
\centering
\caption{%s 正常场景测试用例}
\label{tab:normal_%s}
\begin{tabular}{|c|p{3.5cm}|p{3.5cm}|p{3.5cm}|}
\hline
ID & 描述 & 输入 & 期望输出 \\ \hline
%s
\end{tabular}
\end{table}

\noindent\textbf{异常场景测试用例如表~\ref{tab:abnormal_%s} 所示：}
\begin{table}[h]
\centering
\caption{%s 异常场景测试用例}
\label{tab:abnormal_%s}
\begin{tabular}{|c|p{3.5cm}|p{3.5cm}|p{3.5cm}|}
\hline
ID & 描述 & 输入 & 期望输出 \\ \hline
%s
\end{tabular}
\end{table}

\noindent\textbf{符号说明：}
\begin{itemize}
  \item dx, dy: ego-frame 相对位置 (m)
  \item v\_rx, v\_ry: ego-frame 相对速度 (m/s)
  \item r: 安全圆半径 (m)
  \item B: Barrier 函数值
\end{itemize}
""" % (f["name"], f["name"], f["name"], n_rows, f["name"], f["name"], f["name"], a_rows)


def render(f):
    body = ""
    body += section_requirements(f)
    body += section_algorithm(f)
    body += section_flowchart(f)
    body += section_io(f)
    body += section_metadata(f)
    body += section_arch(f)
    body += section_test(f)
    out = PREAMBLE % f["name"] + r"\begin{document}" + "\n"
    out += cover(f["name"], f["cn"])
    out += body
    out += r"\end{document}" + "\n"
    return out


def main():
    DOC.mkdir(parents=True, exist_ok=True)
    for f in FUNCTIONS:
        sub = DOC / f["name"]
        sub.mkdir(parents=True, exist_ok=True)
        tex_path = sub / (f["name"] + ".tex")
        with open(tex_path, "w", encoding="utf-8") as fp:
            fp.write(render(f))
        print("WROTE", tex_path)


if __name__ == "__main__":
    main()
