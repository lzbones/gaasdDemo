# 🧩 gaasdPrompt - 全流程代码工程管线 Prompt 集合

本仓库包含一套完整的 Prompt 模板，用于指导 AI 完成从 C++ 代码改写 → 测试生成 → 设计文档 → MBD 重构 → MBD 测试的完整闭环。

---

## 📁 目录结构

```
.
├── 00_gaasdPrompt.md         # 全流程编排主入口
├── CppCoding/                # C++ 代码改写与测试
│   ├── 01_cpp_coding.md      # C++ 面向过程代码改写规范
│   └── 02_cpp_testing.md     # C++ 测试生成与验证规范
├── CppDesign/                # 设计文档生成
│   └── 03_design_doc_gen.md  # LaTeX 格式函数设计文档生成
├── MbdRefactor/              # MBD 重构与测试
│   ├── 04_mbd_refactor.md    # FuncModule 架构重构规范
│   └── 05_mbd_testing.md     # MBD 测试生成与验证规范
├── script/                   # 自动化编译与管线脚本
│   ├── compile_latex.py      # LaTeX 自动编译、修正与清理脚本
│   ├── run_pipeline.py       # 自动化管线构建与测试验证脚本
│   └── README.md             # Script 目录说明文档
└── README.md                 # 本说明文件
```

---

## 🚀 快速开始

1. **打开 `00_gaasdPrompt.md`** - 这是全流程编排的主入口文件
2. **勾选需要执行的步骤** - 根据需求选择 Step 01-05
3. **将内容复制给 AI 助手** - AI 会自动加载对应的 Prompt 模板并引导完成每个步骤

---

## 📋 执行流程

| 步骤 | 功能 | 输入 | 输出 |
|------|------|------|------|
| Step 01 | C++ 代码改写 | 原始 C++ 代码 | 面向过程规范的 C++ 代码 |
| Step 02 | C++ 测试生成 | Step 01 输出的代码 | `tests/cppTest/` - 报告、用例与图表 |
| Step 03 | 设计文档生成 | Step 01 输出的代码 | `doc/` - LaTeX 与 PDF 设计文档 |
| Step 04 | MBD 架构重构 | Step 01 输出的代码 | `src/mbd/`, `include/mbd/`, `models/` |
| Step 05 | MBD 测试验证 | Step 04 输出的代码 | `tests/mbdTest/` - 报告、用例与图表 |

---

## 📂 各文件夹说明

### CppCoding/
包含 C++ 面向过程代码改写和测试生成的 Prompt 模板。
- **01_cpp_coding.md**: 定义面向过程编程规范（SSA、单一出口原则等），输出到 `src/cpp/`, `include/cpp/`
- **02_cpp_testing.md**: 生成单元测试、验证报告和可视化脚本，输出到 `tests/cppTest/`

### CppDesign/
包含 LaTeX 格式函数设计文档生成的 Prompt 模板。
- **03_design_doc_gen.md**: 根据源代码自动生成符合企业规范的 LaTeX 设计文档，输出到 `doc/`

### MbdRefactor/
包含 MBD FuncModule 架构重构和测试的 Prompt 模板。
- **04_mbd_refactor.md**: 将原始 C++ 代码重构为 C++20 FuncModule 架构，输出到 `src/mbd/`, `include/mbd/`, `models/`
- **05_mbd_testing.md**: 为 FuncModule 架构代码生成测试用例和验证报告，输出到 `tests/mbdTest/`

### script/
包含用于自动化开发流程、测试验证与文档编译的 Python 脚本。
- **compile_latex.py**: 自动编译 LaTeX 报告为 PDF，并清理编译产生的辅助文件
- **run_pipeline.py**: 一键式构建、测试与生成测试报告和图表，支持多模块的自动化验证
- **README.md**: Script 目录的详细使用说明

---

## 📝 输出产物说明

执行完整流程后，将生成以下产物：

```
project_root/
├── src/                          # 源代码目录
│   ├── cpp/                      # 普通 C++ 源代码（One Function Per File）
│   └── mbd/                      # MBD FuncModule 架构代码
├── include/                      # 头文件目录
│   ├── cpp/                      # 普通 C++ 头文件
│   └── mbd/                      # MBD FuncModule 架构头文件
├── models/                       # MBD 拓扑蓝图（仅 MBD 模块）
│   └── [ModuleName].json
├── tests/                        # 测试相关文件
│   ├── cppTest/                  # C++ 测试结果
│   │   ├── unit/                 # 单元测试代码和用例数据
│   │   ├── verify/               # 程序验证结果
│   │   └── output/               # 可视化输出图表
│   └── mbdTest/                  # MBD 测试结果
│       ├── unit/                 # Traits 级单元测试代码和用例数据
│       ├── verify/               # 架构规范验证结果
│       └── output/               # 可视化输出图表
├── doc/                          # LaTeX 设计文档 (.tex + .pdf)
└── build/                        # 编译输出目录
```

---

## ⚠️ 环境要求

| 工具 | 版本要求 | 用途 |
|------|----------|------|
| **LaTeX** | xelatex (TeX Live / MacTeX) | Step 03 设计文档编译 |
| **Python** | 3.8+ (`~/.ai-env` 虚拟环境) | 编译脚本、测试可视化 |
| **C++ 编译器** | GCC 10+ 或 Clang 12+ (支持 C++20) | MBD 架构代码编译 |
| **CMake** | 3.16+ | 项目构建配置 |

---

## 📖 更多信息

详细的使用说明和注意事项请参阅以下文件：

- **`00_gaasdPrompt.md`** - 全流程编排主入口，包含完整的步骤选择和执行规则
- **`script/README.md`** - Script 目录下各脚本的详细使用说明

### 目录结构说明

普通 C++ 代码和 MBD 架构代码是分离存放的：

| 类型 | 源代码 | 头文件 | 测试文件 |
|------|--------|--------|----------|
| **普通 C++** | `src/cpp/` | `include/cpp/` | `tests/cppTest/` |
| **MBD 架构** | `src/mbd/` | `include/mbd/` | `tests/mbdTest/` |