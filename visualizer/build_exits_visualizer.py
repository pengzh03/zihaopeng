import os
import sys
import argparse
import json
import matplotlib.pyplot as plt


def load_json(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def visualize_case(case_name, layout_path, netlist_path, exit_path=None, save_path=None, label=False, show=False, show_exits=True):
    layout = load_json(layout_path)
    netlist = load_json(netlist_path)
    
    exit_data = None
    if show_exits and exit_path and os.path.exists(exit_path):
        try:
            exit_data = load_json(exit_path)
            print(f"✅ Loaded exit terminals from {exit_path}")
        except Exception as e:
            print(f"⚠️ Failed to load exit file: {e}")
            exit_data = None
    elif show_exits:
        print(f"⚠️ Exit file not found at {exit_path}, skip exit visualization")

    grid_info = layout["grid_info"]
    grid_w = grid_info["grid_max_width"]
    grid_h = grid_info["grid_max_height"]

    def to_grid(x, y):
        gx = x
        gy = y
        return gx, gy

    bottom_bumps = {}
    top_bumps = {}
    for b in layout["bottom_layer"]:
        gx, gy = to_grid(b["grid_coord_x"], b["grid_coord_y"])
        bottom_bumps[b["c4_name"]] = (gx, gy)
    for b in layout["top_layer"]:
        gx, gy = to_grid(b["grid_coord_x"], b["grid_coord_y"])
        top_bumps[b["bump_name"]] = (gx, gy)

    bottom_exits = {}
    top_exits = {}
    if exit_data:
        for item in exit_data:
            bump_name = item["bump_name"]
            exit_x = item["exit_x"]
            exit_y = item["exit_y"]
            bump_layer = item["bump_layer"]
            gx, gy = to_grid(exit_x, exit_y)
            if bump_layer == 0:
                bottom_exits[bump_name] = (gx, gy)
            else:
                top_exits[bump_name] = (gx, gy)

    nx = grid_w
    ny = grid_h

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
        ax.set_xlabel("Grid X")
        ax.set_ylabel("Grid Y")

    ax_b = axes[0]
    draw_grid(ax_b, "Bottom Layer")
    ax_b.scatter([x for x, y in bottom_bumps.values()], 
                 [y for x, y in bottom_bumps.values()], 
                 color='steelblue', s=1, label="Original Bump")
    if label:
        for name, (gx, gy) in bottom_bumps.items():
            ax_b.text(gx + 0.2, gy - 0.2, name, fontsize=6)
    if show_exits and bottom_exits:
        exit_xs = [x for x, y in bottom_exits.values()]
        exit_ys = [y for x, y in bottom_exits.values()]
        ax_b.scatter(exit_xs, exit_ys, color='orange', s=0.2, marker='^', label="Exit Terminal")
        for bump_name, (exit_x, exit_y) in bottom_exits.items():
            if bump_name in bottom_bumps:
                orig_x, orig_y = bottom_bumps[bump_name]
                ax_b.plot([orig_x, exit_x], [orig_y, exit_y], 
                          linestyle='--', color='green', linewidth=0.1, alpha=0.3)
    # ax_b.legend(fontsize=8, loc='upper right')

    ax_t = axes[1]
    draw_grid(ax_t, "Top Layer")
    ax_t.scatter([x for x, y in top_bumps.values()], 
                 [y for x, y in top_bumps.values()], 
                 color='steelblue', s=1, label="Original Bump")
    if label:
        for name, (gx, gy) in top_bumps.items():
            ax_t.text(gx + 0.2, gy - 0.2, name, fontsize=6)
    if show_exits and top_exits:
        exit_xs = [x for x, y in top_exits.values()]
        exit_ys = [y for x, y in top_exits.values()]
        ax_t.scatter(exit_xs, exit_ys, color='orange', s=0.2, marker='^', label="Exit Terminal")
        for bump_name, (exit_x, exit_y) in top_exits.items():
            if bump_name in top_bumps:
                orig_x, orig_y = top_bumps[bump_name]
                ax_t.plot([orig_x, exit_x], [orig_y, exit_y], 
                          linestyle='--', color='green', linewidth=0.1, alpha=0.3)
    # ax_t.legend(fontsize=8, loc='upper right')

    for net in netlist["nets"]:
        bumps = net["bumps"]
        coords = []
        for b in bumps:
            name = b["bump_name"]
            if name in bottom_bumps:
                coords.append((bottom_bumps[name], "bottom"))
            elif name in top_bumps:
                coords.append((top_bumps[name], "top"))

        if len(coords) < 2:
            continue

        layer_types = set([l for (_, l) in coords])
        if len(layer_types) == 1:
            layer = coords[0][1]
            ax = ax_b if layer == "bottom" else ax_t
            (x1, y1), _ = coords[0]
            for (x2, y2), _ in coords[1:]:
                ax.plot([x1, x2], [y1, y2], linestyle='dotted', color='red', linewidth=0.1, alpha=0.5)
        else:
            ((x1, y1), l1), ((x2, y2), l2) = coords[:2]
            if l1 == "bottom" and l2 == "top":
                ax_b.plot([x1, x2], [y1, y2], linestyle='dotted', color='purple', linewidth=0.1, alpha=0.5)
                ax_t.plot([x1, x2], [y1, y2], linestyle='dotted', color='purple', linewidth=0.1, alpha=0.5)
            elif l1 == "top" and l2 == "bottom":
                ax_b.plot([x2, x1], [y2, y1], linestyle='dotted', color='purple', linewidth=0.1, alpha=0.5)
                ax_t.plot([x2, x1], [y2, y1], linestyle='dotted', color='purple', linewidth=0.1, alpha=0.5)

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"✅ Visualization saved to {save_path}")
    if show:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize 2.5D routing cases (with Exit Terminals)")
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
    exit_dir = "../output"

    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)

    exit_path = os.path.join(script_dir, exit_dir, f"{case_name}_exit_terminals.json")
    exit_path = os.path.normpath(exit_path)

    benchmark_dir = os.path.join(script_dir, "../OpenDACS2025-contest4-Benchmark")
    benchmark_dir = os.path.normpath(benchmark_dir)

    layout_path = os.path.join(benchmark_dir, f"{case_name}_grid_layout.json")
    netlist_path = os.path.join(benchmark_dir, f"{case_name}_netlist.json")

    if not os.path.isfile(layout_path):
        print(f"Error: Layout file not found: {layout_path}")
        sys.exit(1)
    if not os.path.isfile(netlist_path):
        print(f"Error: Netlist file not found: {netlist_path}")
        sys.exit(1)

    pic_path = os.path.join(script_dir, "pic")
    os.makedirs(pic_path, exist_ok=True)
    save_path = os.path.join(pic_path, f"{case_name}_build_exits.pdf")

    visualize_case(
        case_name, 
        layout_path, 
        netlist_path, 
        exit_path=exit_path,
        save_path=save_path, 
        label=args.label, 
        show=args.show,
        show_exits=True
    )