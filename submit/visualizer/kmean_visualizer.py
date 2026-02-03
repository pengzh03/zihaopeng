import os
import sys
import argparse
import json
import matplotlib.pyplot as plt


def load_json(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def read_vector_data(file_path):
    data = {}  # 存储结果：key=分组标识（int），value=该组所有向量（list of list）
    bound = {}
    current_group = None  # 当前分组标识

    switch = False
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()  # 去除首尾空白（换行符、空格）
            if not line:
                continue  # 跳过空行

            if '----' in line:
                switch = True
                continue

            if not switch:
                # 判断是否为分组标识行（纯数字）
                if line.isdigit():
                    current_group = int(line)
                    data[current_group] = []  # 初始化该分组的向量列表
                else:
                    # 解析向量行：按制表符分割，再提取每个向量的数值
                    vector_strs = line.split('\t')  # 分割多个向量
                    for vec in vector_strs:
                        data[current_group].append(eval(vec))
            else:
                # 判断是否为分组标识行（纯数字）
                if line.isdigit():
                    current_group = int(line)
                else:
                    bound[current_group] = eval(line)

    return data, bound


def visualize_case(case_name, layout_path, netlist_path, save_path=None, label=False, show=False):
    layout = load_json(layout_path)

    grid_info = layout["grid_info"]
    grid_len = grid_info["grid_length"]
    grid_w = grid_info["grid_max_width"]
    grid_h = grid_info["grid_max_height"]

    def to_grid(x, y):
        gx = int(round(x / grid_len))
        gy = int(round(y / grid_len))
        return gx, gy

    bottom_bumps = {}
    for b in layout["bottom_layer"]:
        gx, gy = to_grid(b["grid_coord_x"], b["grid_coord_y"])
        bottom_bumps[b["c4_name"]] = (gx, gy)

    top_bumps = {}
    for b in layout["top_layer"]:
        gx, gy = to_grid(b["grid_coord_x"], b["grid_coord_y"])
        top_bumps[b["bump_name"]] = (gx, gy)

    nx = grid_w // grid_len
    ny = grid_h // grid_len

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle(f"Routing Case: {case_name}", fontsize=16, weight='bold')

    def draw_grid(ax, title):
        for x in range(nx + 1):
            ax.plot([x, x], [0, ny], color="lightgray", linewidth=0.05)
        for y in range(ny + 1):
            ax.plot([0, nx], [y, y], color="lightgray", linewidth=0.05)
        ax.set_aspect('equal')
        ax.set_xlim(0, nx)
        ax.set_ylim(0, ny)
        ax.set_title(title)
        ax.invert_yaxis()
        ax.set_xlabel("Grid X")
        ax.set_ylabel("Grid Y")

    ax_b = axes[0]
    draw_grid(ax_b, "Bottom Layer")
    for name, (gx, gy) in bottom_bumps.items():
        # ax_b.scatter(gx, gy, color='steelblue', s=1)
        if label:
            ax_b.text(gx + 0.2, gy - 0.2, name, fontsize=6)

    ax_t = axes[1]
    draw_grid(ax_t, "Top Layer")
    for name, (gx, gy) in top_bumps.items():
        # ax_t.scatter(gx, gy, color='steelblue', s=1)
        if label:
            ax_t.text(gx + 0.2, gy - 0.2, name, fontsize=6)

    net_data, net_bound = read_vector_data(netlist_path)
    max_classes = len(net_data)
    cmap1 = plt.cm.get_cmap('tab20', 20)
    cmap2 = plt.cm.get_cmap('tab20b', 20)
    cmap3 = plt.cm.get_cmap('tab20c', 20)
    colors = list(cmap1.colors) + list(cmap2.colors) + list(cmap3.colors)
    colors = colors[:max_classes]  # 截断到需要的数量
    for c in net_data:
        color_idx = c - 1  # 根据实际类别编号起始值调整
        color = colors[color_idx]
        bound = net_bound[c]
        bus_y, brs_x, bds_y, bls_x = bound[0][0], bound[0][1], bound[0][2], bound[0][3]
        bue_y, bre_x, bde_y, ble_x = bound[1][0], bound[1][1], bound[1][2], bound[1][3]
        bound_points_start = [(to_grid(bls_x, bus_y)), to_grid(brs_x, bus_y), to_grid(brs_x, bds_y), to_grid(bls_x, bds_y), (to_grid(bls_x, bus_y))]
        bound_points_end = [(to_grid(ble_x, bue_y)), to_grid(bre_x, bue_y), to_grid(bre_x, bde_y), to_grid(ble_x, bde_y), (to_grid(ble_x, bue_y))]
        bs_x = [p[0] for p in bound_points_start]
        bs_y = [p[1] for p in bound_points_start]
        be_x = [p[0] for p in bound_points_end]
        be_y = [p[1] for p in bound_points_end]
        for vec in net_data[c]:
            sx, sy = to_grid(vec[0][0], vec[0][1])
            sl = vec[0][2]
            ex, ey = to_grid(vec[1][0], vec[1][1])
            el = vec[1][2]
            if sl == 0:
                ax_b.scatter(sx, sy, s=1, color=color)
                ax_b.plot(bs_x, bs_y, linestyle='solid', color=color, linewidth=1)
            else:
                ax_t.scatter(sx, sy, s=1, color=color)
                ax_t.plot(bs_x, bs_y, linestyle='solid', color=color, linewidth=1)
            if el == 0:
                ax_b.scatter(ex, ey, s=1, color=color)
                ax_b.plot(be_x, be_y, linestyle='solid', color=color, linewidth=1)
            else:
                ax_t.scatter(ex, ey, s=1, color=color)
                ax_t.plot(be_x, be_y, linestyle='solid', color=color, linewidth=1)
            if not (sl == 0 and el == 0):
                ax_t.plot([sx, ex], [sy, ey], linestyle='solid', color=color, linewidth=0.5, alpha=0.5)
            if not (sl == 1 and el == 1):
                ax_b.plot([sx, ex], [sy, ey], linestyle='solid', color=color, linewidth=0.5, alpha=0.5)


    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"✅ Visualization saved to {save_path}")
    if show:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize 2.5D routing cases")
    parser.add_argument("case", type=str, help="Name of the routing case")
    parser.add_argument(
        "--label",
        action="store_true",
        help="Show bump names in the visualization (default: False)"
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the visualization interactively (default: False)"
    )
    args = parser.parse_args()

    case_name = args.case

    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)

    benchmark_dir = os.path.join(script_dir, "../OpenDACS2025-contest4-Benchmark")
    benchmark_dir = os.path.normpath(benchmark_dir)

    layout_path = os.path.join(benchmark_dir, f"{case_name}_grid_layout.json")
    netlist_path = f"output/kmeans_result.txt"

    if not os.path.isfile(layout_path):
        print(f"Error: Layout file not found: {layout_path}")
        sys.exit(1)
    if not os.path.isfile(netlist_path):
        print(f"Error: Netlist file not found: {netlist_path}")
        sys.exit(1)

    pic_path = os.path.join(script_dir, "pic")
    os.makedirs(pic_path, exist_ok=True)
    save_path = os.path.join(pic_path, f"{case_name}_grid_visualization_kmean.pdf")

    visualize_case(
        case_name,
        layout_path,
        netlist_path,
        save_path,
        label=args.label,
        show=args.show
    )
