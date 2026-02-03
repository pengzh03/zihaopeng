import os
import sys
import argparse
import json
import re
import matplotlib.pyplot as plt


def load_json(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)
    
def get_layer_num(layer_name, layer_count):
    if layer_name.lower() == "bottom":
        return 0
    if layer_name.lower() == "top":
        return layer_count - 1
    num = re.findall(r'\d+', layer_name)
    if num:
        num = int(num[0])
        return num
    raise ValueError(f"Invalid layer name: {layer_name}")


def visualize_case(case_name, layer_count, layout_path, netlist_path, routing_result_path=None, save_path=None, label=False, show=False):
    layout = load_json(layout_path)
    netlist = load_json(netlist_path)
    
    routing_result = None
    if routing_result_path and os.path.exists(routing_result_path):
        routing_result = load_json(routing_result_path)

    grid_info = layout["grid_info"]
    # grid_len = grid_info["grid_length"]
    grid_w = grid_info["grid_max_width"]
    grid_h = grid_info["grid_max_height"]

    def to_grid(x, y):
        gx = x
        gy = y
        return gx, gy

    bottom_bumps = {}
    for b in layout["bottom_layer"]:
        gx, gy = to_grid(b["grid_coord_x"], b["grid_coord_y"])
        bottom_bumps[b["c4_name"]] = (gx, gy)

    top_bumps = {}
    for b in layout["top_layer"]:
        gx, gy = to_grid(b["grid_coord_x"], b["grid_coord_y"])
        top_bumps[b["bump_name"]] = (gx, gy)

    nx = grid_w
    ny = grid_h

    fig, axes = plt.subplots(1, layer_count, figsize=(14, 7))
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
        # ax.invert_yaxis()
        ax.set_xlabel("Grid X")
        ax.set_ylabel("Grid Y")

    ax_b = axes[0]
    draw_grid(ax_b, "Bottom Layer")
    for name, (gx, gy) in bottom_bumps.items():
        ax_b.scatter(gx, gy, color='steelblue', s=1)
        if label:
            ax_b.text(gx + 0.2, gy - 0.2, name, fontsize=6)

    for i in range(1, layer_count - 1):
        draw_grid(axes[i], f"M{i} Layer")

    ax_t = axes[layer_count - 1]
    draw_grid(ax_t, "Top Layer")
    for name, (gx, gy) in top_bumps.items():
        ax_t.scatter(gx, gy, color='steelblue', s=1)
        if label:
            ax_t.text(gx + 0.2, gy - 0.2, name, fontsize=6)

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

        if net["net_name"] == "net_0":
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

    if routing_result:
        route_colors = ['purple', 'orange', 'green', 'red', 'brown']
        route_linewidth = 0.05
        
        net_iter = 0
        for net_name, segments in routing_result.items():
            if net_name != "net_0":
                continue
            for seg in segments:
                start = seg["start_grid_coordinate"]
                end = seg["end_grid_coordinate"]
                
                start_x, start_y, start_layer = start[0], start[1], start[2]
                end_x, end_y, end_layer = end[0], end[1], end[2]
                
                if start_layer != end_layer:
                    start_layer_num = get_layer_num(start_layer, layer_count)
                    end_layer_num = get_layer_num(end_layer, layer_count)
                    for layer_i in range(start_layer_num, end_layer_num + 1):
                        ax = axes[layer_i]
                        ax.plot(
                            [start_x, start_x],
                            [start_y, start_y],
                            linestyle='-',
                            color=route_colors[net_iter % len(route_colors)],
                            linewidth=route_linewidth,
                            alpha=0.8
                        )
                    continue
                                
                if start_layer.lower() == "bottom":
                    ax = ax_b
                elif start_layer.lower() == "top":
                    ax = ax_t
                else:
                    for i in range(1, layer_count - 1):
                        if start_layer == f"M{i}":
                            ax = axes[i]
                            break

                ax.plot(
                    [start_x, end_x], 
                    [start_y, end_y], 
                    linestyle='-', 
                    color=route_colors[net_iter % len(route_colors)], 
                    linewidth=route_linewidth, 
                    alpha=0.8
                )
            net_iter += 1

        handles, labels = ax_b.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        if by_label:
            ax_b.legend(by_label.values(), by_label.keys(), fontsize=8, loc='upper right')
        

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"✅ Visualization saved to {save_path}")
    if show:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize 2.5D routing cases with routing results")
    parser.add_argument("case", type=str, help="Name of the routing case")
    parser.add_argument("layer", type=int, help="Total layer count (including top and bottom layers)")
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
    layer_count = args.layer

    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)

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
    save_path = os.path.join(pic_path, f"{case_name}_grid_visualization_2.pdf")

    result_dir = os.path.join(script_dir, "../output")
    routing_result_path = os.path.join(result_dir, f"{case_name}_result.json")

    visualize_case(
        case_name,
        layer_count,
        layout_path, 
        netlist_path, 
        routing_result_path=routing_result_path,
        save_path=save_path, 
        label=args.label, 
        show=args.show
    )