import os
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle

def read_pairteams_from_file(filename):
    pair_teams = []
    current_pt = {}
    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line == '---':
            if current_pt:
                pair_teams.append(current_pt)
                current_pt = {}
            continue
        if ': ' in line:
            key, value = line.split(': ', 1)
            try:
                value = int(value)
            except ValueError:
                print(f"Warning: Invalid number for key '{key}', value '{value}'")
                continue
            if key == 'PairTeamID':
                current_pt['id'] = value
            elif key == 'Layer':
                current_pt['layer'] = value
            elif key == 'StartLeft':
                current_pt.setdefault('start', {})['left'] = value
            elif key == 'StartRight':
                current_pt.setdefault('start', {})['right'] = value
            elif key == 'StartUp':
                current_pt.setdefault('start', {})['up'] = value
            elif key == 'StartDown':
                current_pt.setdefault('start', {})['down'] = value
            elif key == 'EndLeft':
                current_pt.setdefault('end', {})['left'] = value
            elif key == 'EndRight':
                current_pt.setdefault('end', {})['right'] = value
            elif key == 'EndUp':
                current_pt.setdefault('end', {})['up'] = value
            elif key == 'EndDown':
                current_pt.setdefault('end', {})['down'] = value
    if current_pt:
        pair_teams.append(current_pt)
    return pair_teams

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize 2.5D routing cases with layer assignments")
    parser.add_argument("case", type=str, help="Name of the routing case")
    args = parser.parse_args()
    case_name = args.case
    script_path = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_path)
    result_dir = os.path.join(script_dir, "../output")
    input_path = os.path.join(result_dir, f"{case_name}_layer_assignment.txt")

    pair_teams = read_pairteams_from_file(input_path)

    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False

    layer_colors = {
        0: '#1f77b4',
        1: '#ff7f0e',
        2: '#2ca02c',
        3: '#d62728',
        4: '#9467bd',
        5: '#8c564b',
        6: '#e377c2',
        7: '#7f7f7f',
        8: '#bcbd22',
        9: '#17becf',
        10: '#aec7e8',
        11: '#ffbb78',
        12: '#98df8a',
        13: '#ff9896',
        14: '#c5b0d5',
        15: '#c49c94',
        16: '#f7b6d3',
        17: '#c7c7c7',
        18: '#dbdb8d',
        19: '#9edae5'
    }
    line_styles = {'start': '-', 'end': '--'}

    fig, ax = plt.subplots(1, 1, figsize=(16, 12))

    for pt in pair_teams:
        pt_id = pt["id"]
        layer = pt["layer"]
        color = layer_colors[layer]

        start = pt["start"]
        start_width = start["right"] - start["left"]
        start_height = start["down"] - start["up"]
        start_rect = Rectangle(
            (start["left"], start["up"]),
            start_width, start_height,
            linewidth=2,
            edgecolor=color,
            linestyle=line_styles['start'],
            facecolor='none',
            label=f'layer {layer}' if f'layer {layer}' not in ax.get_legend_handles_labels()[1] else ""
        )
        ax.add_patch(start_rect)

        end = pt["end"]
        end_width = end["right"] - end["left"]
        end_height = end["down"] - end["up"]
        end_rect = Rectangle(
            (end["left"], end["up"]),
            end_width, end_height,
            linewidth=2,
            edgecolor=color,
            linestyle=line_styles['end'],
            facecolor='none'
        )
        ax.add_patch(end_rect)
        
        start_center_x = start["left"] + start_width / 2
        start_center_y = start["up"] + start_height / 2
        ax.text(start_center_x, start_center_y, f'PT{pt_id}', 
                ha='center', va='center', fontsize=8, fontweight='bold')
        
        end_center_x = end["left"] + end_width / 2
        end_center_y = end["up"] + end_height / 2
        ax.text(end_center_x, end_center_y, f'PT{pt_id}', 
                ha='center', va='center', fontsize=8, fontweight='bold')

    ax.autoscale_view()
    ax.set_xlabel('X', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y', fontsize=12, fontweight='bold')
    ax.set_title('PairTeam Layer Assignment\n (solid line: start group, dashed line: end group, color: layer)', 
                fontsize=14, fontweight='bold', pad=20)

    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], linestyle='-', color='black', linewidth=2, label='start group'),
        Line2D([0], [0], linestyle='--', color='black', linewidth=2, label='end group')
    ]
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles + legend_elements, labels + ['start group', 'end group'], 
            loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, linestyle=':')
    ax.invert_yaxis()

    plt.tight_layout()
    pic_path = os.path.join(script_dir, "pic")
    os.makedirs(pic_path, exist_ok=True)
    save_path = os.path.join(pic_path, f"{case_name}_layer_assignment.pdf")
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
