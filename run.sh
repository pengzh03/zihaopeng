#!/bin/bash
set -e  # 出错立即退出，防止错误累积

# ====================== 只需改这里的路径 ======================
# 输入文件：芯片布局和网表信息
GRID_LAYOUT="/home/pengzh/submit/OpenDACS2025-contest4-Benchmark/C2IO1_grid_layout.json"
NETLIST="/home/pengzh/submit/OpenDACS2025-contest4-Benchmark/C2IO1_netlist.json"

# 输出目录：存放所有结果
RESULT_DIR="/home/pengzh/submit/output/test"
mkdir -p "${RESULT_DIR}"  # 如果目录不存在则创建

# 输出文件路径
ROUTING_RESULT="${RESULT_DIR}/routing_result.json"    # 你的布线算法输出
REPORT="${RESULT_DIR}/evaluation_report.json"         # 评测脚本生成的评分报告
EVALUATOR="/home/pengzh/submit/evaluator.py"          # 评测脚本位置

# ====================== VTune 性能分析配置 ======================
# VTune 原始数据保存目录（包含采样的所有详细信息）
VTUNE_RESULT_DIR="${RESULT_DIR}/vtune_hotspots"

# VTune 最终文本报告保存路径（精简版）
VTUNE_REPORT_FILE="${RESULT_DIR}/vtune_hotspots_report.txt"

# VTune 一页纸摘要报告（快速判断瓶颈类型）
VTUNE_SUMMARY_FILE="${RESULT_DIR}/vtune_summary.txt"

# ====================== 正式执行流程 ======================

# 第一步：清理旧的编译文件
# 目的：确保是全新编译，防止旧的 .o 文件干扰
echo "1. 清理旧文件..."
make clean

# 第二步：编译项目
# 重要前提：请确保 Makefile 里的 CFLAGS/CXXFLAGS 同时包含了 -O3 (优化) 和 -g (调试信息)
echo "2. 编译代码..."
make all

# 新增：自动删除旧的 VTune 结果目录，避免报错
if [ -d "${VTUNE_RESULT_DIR}" ]; then
    echo "   清理旧的 VTune 数据..."
    rm -rf "${VTUNE_RESULT_DIR}"
fi

# 第三步：用 VTune 启动并监控布线程序
# 核心命令解释：
#   -collect hotspots: 采集类型为“CPU热点分析”（最常用，开销最小）
#   -result-dir:       指定 VTune 原始数据的保存位置
#   --:                分隔符，后面跟你的程序及其参数
echo "3. 运行 VTune 热点分析..."
vtune -collect hotspots \
      -result-dir "${VTUNE_RESULT_DIR}" \
      -- ./build/router "${GRID_LAYOUT}" "${NETLIST}" "${ROUTING_RESULT}"

# 第四步：运行评测脚本
# 目的：检查布线结果的正确性，并生成跑分
echo "4. 运行评测程序..."
python3 $EVALUATOR $GRID_LAYOUT $NETLIST $ROUTING_RESULT $REPORT

# 第五步：生成 VTune 分析报告（无需 GUI，服务器端直接看）
echo ""
echo "5. 正在生成 VTune 分析报告..."

# 5.1 生成一页纸摘要（先看这个！）
# 内容：总耗时、CPU利用率、瓶颈类型定性（CPU Bound/Memory Bound）
vtune -report summary \
      -result-dir "${VTUNE_RESULT_DIR}" \
      -format text \
      > "${VTUNE_SUMMARY_FILE}"

# 5.2 生成 Top 10 热点函数列表（后看这个！）
vtune -report hotspots \
      -result-dir "${VTUNE_RESULT_DIR}" \
      -limit 10 \
      -format text \
      > "${VTUNE_REPORT_FILE}"

# ====================== 结束提示 ======================
echo ""
echo "========================================"
echo "             全部流程完成！"
echo "========================================"
echo "1. 布线结果：${ROUTING_RESULT}"
echo "2. 评测报告：${REPORT}"
echo ""
echo "VTune 分析结果（重点看这两个）："
echo "【第一步】快速总览（判断瓶颈类型）："
echo " cat ${VTUNE_SUMMARY_FILE}"
echo " scp -P 24389 -r pengzh@118.26.161.197:${VTUNE_SUMMARY_FILE} "D:\桌面""
echo ""
echo "【第二步】定位具体函数（知道改哪里）："
echo " cat ${VTUNE_REPORT_FILE}"
echo " scp -P 24389 -r pengzh@118.26.161.197:${VTUNE_REPORT_FILE} "D:\桌面""
echo "========================================"
