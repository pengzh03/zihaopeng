#include "a_star.h"
#include <omp.h>
#include <mutex>

AStarNode::AStarNode(int x_, int y_, int layer_)
    : x(x_), y(y_), layer(layer_), g(0), h(0), f(0), parent(nullptr) {}


bool AStarNode::operator>(const AStarNode& other) const {
    return f > other.f;
}


std::string get_coord_node_key(int x, int y, int layer) {
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(layer);
}


std::string get_node_key(float x, float y, int layer) {
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(layer);
}


int grid_distance(int x1, int y1, int x2, int y2) {
    return std::max(abs(x1 - x2), abs(y1 - y2));
}

//====================================Single Thread====================================

std::vector<AStarNode> get_neighbors(const AStarNode& current, int max_width, int max_height, int min_width, int min_height, const std::unordered_set<std::string>& occupied_set, int end_x, int end_y, int end_layer) {
    std::vector<AStarNode> neighbors;
    std::vector<std::pair<int, int>> dirs = {
        {0, 1},
        {0, -1},
        {1, 0},
        {-1, 0},
        {1, 1},
        {1, -1},
        {-1, 1},
        {-1, -1}
    };

    for (const auto& dir : dirs) {
        int dx = dir.first;
        int dy = dir.second;

        int new_x = current.x + dx;
        int new_y = current.y + dy;

        if (new_x >= min_width && new_x <= max_width && new_y >= min_height && new_y <= max_height) {
            std::string coord = get_node_key(new_x, new_y, end_layer);
            if (occupied_set.find(coord) != occupied_set.end()) {
                if (!(new_x == end_x && new_y == end_y)) {
                    continue;
                }
            }
            if (dx != 0 && dy != 0) {
                if (current.parent != nullptr) {
                    if ((new_x == current.parent->x && current.y == current.parent->y) || (new_y == current.parent->y && current.x == current.parent->x)) {
                        continue;
                    }
                }
                float mid_x = static_cast<float>(current.x) + static_cast<float>(dx) * 0.5f;
                float mid_y = static_cast<float>(current.y) + static_cast<float>(dy) * 0.5f;
                std::string key = get_node_key(mid_x, mid_y, end_layer);
                // // std::cout << "==> check key: " << key << std::endl;
                if (occupied_set.find(key) != occupied_set.end()) {
                    continue;
                }
            } else {
                if (dx != 0) {
                    if (current.parent != nullptr) {
                        if (new_x == current.parent->x) {
                            continue;
                        }
                    }
                } else if (dy != 0) {
                    if (current.parent != nullptr) {
                        if (new_y == current.parent->y) {
                            continue;
                        }
                    }
                }
            }
            neighbors.emplace_back(new_x, new_y, end_layer);
        }
    }
    return neighbors;
}

bool is_point_in_convex_polygon(double x, double y, const std::vector<Point>& edges) {
    int n = edges.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Point& p1 = edges[i];
        const Point& p2 = edges[j];
        double cross = (p2.first - p1.first) * (y - p1.second) - (p2.second - p1.second) * (x - p1.first);
        double dist_p1 = sqrt(pow(x - p1.first, 2) + pow(y - p1.second, 2));
        double dist_p2 = sqrt(pow(x - p2.first, 2) + pow(y - p2.second, 2));
        double dist_p1p2 = sqrt(pow(p2.first - p1.first, 2) + pow(p2.second - p1.second, 2));
        if (fabs(cross) < 1e-9 && (dist_p1 + dist_p2 - dist_p1p2) < 1e-9) {
            return true;
        }
        if (((p1.second > y) != (p2.second > y)) && 
            (x < (p2.first - p1.first) * (y - p1.second) / (p2.second - p1.second) + p1.first)) {
            inside = !inside;
        }
    }
    return inside;
}

std::vector<AStarNode> get_neighbors_intersect(const AStarNode& current, const std::vector<Point>& edges, const std::unordered_set<std::string>& occupied_set, int end_x, int end_y, int end_layer) {
    std::vector<AStarNode> neighbors;
    std::vector<std::pair<int, int>> dirs = {
        {0, 1},
        {0, -1},
        {1, 0},
        {-1, 0},
        {1, 1},
        {1, -1},
        {-1, 1},
        {-1, -1}
    };

    for (const auto& dir : dirs) {
        int dx = dir.first;
        int dy = dir.second;

        int new_x = current.x + dx;
        int new_y = current.y + dy;

        if (is_point_in_convex_polygon(new_x, new_y, edges)) {
            std::string coord = get_node_key(new_x, new_y, end_layer);
            if (occupied_set.find(coord) != occupied_set.end()) {
                if (!(new_x == end_x && new_y == end_y)) {
                    continue;
                }
            }
            if (dx != 0 && dy != 0) {
                if (current.parent != nullptr) {
                    if ((new_x == current.parent->x && current.y == current.parent->y) || (new_y == current.parent->y && current.x == current.parent->x)) {
                        continue;
                    }
                }
                float mid_x = static_cast<float>(current.x) + static_cast<float>(dx) * 0.5f;
                float mid_y = static_cast<float>(current.y) + static_cast<float>(dy) * 0.5f;
                std::string key = get_node_key(mid_x, mid_y, end_layer);
                // // std::cout << "==> check key: " << key << std::endl;
                if (occupied_set.find(key) != occupied_set.end()) {
                    continue;
                }
            } else {
                if (dx != 0) {
                    if (current.parent != nullptr) {
                        if (new_x == current.parent->x) {
                            continue;
                        }
                    }
                } else if (dy != 0) {
                    if (current.parent != nullptr) {
                        if (new_y == current.parent->y) {
                            continue;
                        }
                    }
                }
            }
            neighbors.emplace_back(new_x, new_y, end_layer);
        }
    }
    return neighbors;
}

std::vector<AStarNode> reconstruct_path(AStarNode* end_node, std::unordered_set<std::string>& occupied_set) {
    std::vector<AStarNode> path;
    AStarNode* current = end_node;
    AStarNode* last = end_node;
    while (current != nullptr) {
        path.push_back(*current);
        std::string last_node_key = get_node_key(current->x, current->y, current->layer);
        if (occupied_set.find(last_node_key) == occupied_set.end()) {
            occupied_set.insert(last_node_key);
        }
        if ((last->x != current->x) && (last->y != current->y)) {
            float mid_x = static_cast<float>(last->x + current->x) * 0.5f;
            float mid_y = static_cast<float>(last->y + current->y) * 0.5f;
            std::string mid_node_key = get_node_key(mid_x, mid_y, current->layer);
            if (occupied_set.find(mid_node_key) == occupied_set.end()) {
                occupied_set.insert(mid_node_key);
            } else {
                // // std::cout << "Occupied node encountered in a star: " << mid_node_key << std::endl;
            }
            // // std::cout << "==> insert key: " << get_node_key(mid_x, mid_y, current->layer) << std::endl;
        }
        last = current;
        current = current->parent;
    }
    return path;
}

AStarResult a_star_find_path_to_exit(
    const Bump& bump,
    int max_width,
    int max_height,
    int min_width,
    int min_height,
    std::unordered_set<std::string>& occupied_set
) {
    const ExitTerminal& target_exit = bump.exit_terminal;
    int end_x = target_exit.grid_coord_x;
    int end_y = target_exit.grid_coord_y;
    int end_layer = target_exit.layer;
    // // std::cout << "A* routing to exit for bump " << bump.net_name << " from (" << bump.grid_coord_x << ", " << bump.grid_coord_y << ", " << bump.layer << ") to exit (" << end_x << ", " << end_y << ", " << end_layer << ")\n";
    // // std::cout << "width range: [" << min_width << ", " << max_width << "], height range: [" << min_height << ", " << max_height << "]\n";

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, int> best_g;
    std::unordered_set<AStarNode*> all_allocated_nodes;
    AStarResult routing_result;

    AStarNode start_node(bump.grid_coord_x, bump.grid_coord_y, end_layer);
    start_node.h = grid_distance(
        start_node.x, start_node.y,
        end_x, end_y
    );
    start_node.f = start_node.g + start_node.h;
    open_list.push(start_node);
    best_g[get_coord_node_key(start_node.x, start_node.y, start_node.layer)] = 0;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        // // std::cout << "current g: " << current.g << "\n";
        open_list.pop();
        std::string current_key = get_coord_node_key(current.x, current.y, current.layer);

        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            // // std::cout << "Path found with length: " << current.g << "\n";
            routing_result.path = reconstruct_path(&current, occupied_set);
            routing_result.nodes_to_delete = std::move(all_allocated_nodes);
            return routing_result;
        }

        if (closed_set.count(current_key) && best_g[current_key] < current.g) {
            continue;
        }
        closed_set.insert(current_key);

        std::vector<AStarNode> neighbors = get_neighbors(current, max_width, max_height, min_width, min_height, occupied_set, end_x, end_y, end_layer);
        for (AStarNode& neighbor : neighbors) {
            std::string neighbor_key = get_coord_node_key(neighbor.x, neighbor.y, neighbor.layer);
            if (closed_set.count(neighbor_key)) {
                continue;
            }

            int tentative_g = current.g + 1;
            if (!best_g.count(neighbor_key) || tentative_g < best_g[neighbor_key]) {
                neighbor.parent = new AStarNode(current);
                all_allocated_nodes.insert(neighbor.parent);
                neighbor.g = tentative_g;
                neighbor.h = grid_distance(
                    neighbor.x, neighbor.y,
                    end_x, end_y
                );
                neighbor.f = neighbor.g + neighbor.h;

                open_list.push(neighbor);
                best_g[neighbor_key] = tentative_g;
            }
        }
    }

    // // std::cout << "No path found for bump " << bump.net_name << " from (" << bump.grid_coord_x << ", " << bump.grid_coord_y << ", " << bump.layer << ") to exit (" << end_x << ", " << end_y << ", " << end_layer << ")\n";
    // // std::cout << "    routing range: width range: [" << min_width << ", " << max_width << "], height range: [" << min_height << ", " << max_height << "]\n";
    routing_result.path = {};
    routing_result.nodes_to_delete = std::move(all_allocated_nodes);
    return routing_result;
}

AStarResult a_star_find_path_to_exit_intersect(
    const Bump& bump,
    const std::vector<Point>& edges,
    std::unordered_set<std::string>& occupied_set
) {
    const ExitTerminal& target_exit = bump.exit_terminal;
    int end_x = target_exit.grid_coord_x;
    int end_y = target_exit.grid_coord_y;
    int end_layer = target_exit.layer;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, int> best_g;
    std::unordered_set<AStarNode*> all_allocated_nodes;
    AStarResult routing_result;

    AStarNode start_node(bump.grid_coord_x, bump.grid_coord_y, end_layer);
    start_node.h = grid_distance(
        start_node.x, start_node.y,
        end_x, end_y
    );
    start_node.f = start_node.g + start_node.h;
    open_list.push(start_node);
    best_g[get_coord_node_key(start_node.x, start_node.y, start_node.layer)] = 0;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        // // std::cout << "current g: " << current.g << "\n";
        open_list.pop();
        std::string current_key = get_coord_node_key(current.x, current.y, current.layer);

        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            // // std::cout << "Path found with length: " << current.g << "\n";
            routing_result.path = reconstruct_path(&current, occupied_set);
            routing_result.nodes_to_delete = std::move(all_allocated_nodes);
            return routing_result;
        }

        if (closed_set.count(current_key) && best_g[current_key] < current.g) {
            continue;
        }
        closed_set.insert(current_key);

        std::vector<AStarNode> neighbors = get_neighbors_intersect(current, edges, occupied_set, end_x, end_y, end_layer);
        for (AStarNode& neighbor : neighbors) {
            std::string neighbor_key = get_coord_node_key(neighbor.x, neighbor.y, neighbor.layer);
            if (closed_set.count(neighbor_key)) {
                continue;
            }

            int tentative_g = current.g + 1;
            if (!best_g.count(neighbor_key) || tentative_g < best_g[neighbor_key]) {
                neighbor.parent = new AStarNode(current);
                all_allocated_nodes.insert(neighbor.parent);
                neighbor.g = tentative_g;
                neighbor.h = grid_distance(
                    neighbor.x, neighbor.y,
                    end_x, end_y
                );
                neighbor.f = neighbor.g + neighbor.h;

                open_list.push(neighbor);
                best_g[neighbor_key] = tentative_g;
            }
        }
    }

    // // std::cout << "No path found for bump " << bump.net_name << " from (" << bump.grid_coord_x << ", " << bump.grid_coord_y << ", " << bump.layer << ") to exit (" << end_x << ", " << end_y << ", " << end_layer << ")\n";
    // // std::cout << "    routing range:\n";
    // for (const auto& edge : edges) {
    //     // std::cout << "        (" << edge.first << ", " << edge.second << "); ";
    // }
    // // std::cout << "\n";
    routing_result.path = {};
    routing_result.nodes_to_delete = std::move(all_allocated_nodes);
    return routing_result;
}

bool is_same_direction(int dx1, int dy1, int dl1, int dx2, int dy2, int dl2) {
    return (dx1 == dx2) && (dy1 == dy2) && (dl1 == dl2);
}

std::vector<std::pair<ResultPoint, ResultPoint>> split_to_segments(const std::vector<AStarNode>& path) {
    std::vector<std::pair<ResultPoint, ResultPoint>> segments;
    if (path.size() < 2) {
        return segments;
    }

    ResultPoint current_start = {path[0].x, path[0].y, path[0].layer};
    int prev_dx = path[1].x - path[0].x;
    int prev_dy = path[1].y - path[0].y;
    int prev_dl = path[1].layer - path[0].layer;

    for (size_t i = 2; i < path.size(); ++i) {
        const AStarNode& prev_node = path[i-1];
        const AStarNode& curr_node = path[i];

        int curr_dx = curr_node.x - prev_node.x;
        int curr_dy = curr_node.y - prev_node.y;
        int curr_dl = curr_node.layer - prev_node.layer;

        if (!is_same_direction(prev_dx, prev_dy, prev_dl, curr_dx, curr_dy, curr_dl)) {
            ResultPoint new_node = {prev_node.x, prev_node.y, prev_node.layer};
            segments.emplace_back(current_start, new_node);
            current_start = new_node;
            prev_dx = curr_dx;
            prev_dy = curr_dy;
            prev_dl = curr_dl;
        }
    }

    const AStarNode& last_node = path.back();
    ResultPoint end_point = {last_node.x, last_node.y, last_node.layer};

    int dx = current_start.x - end_point.x;
    int dy = current_start.y - end_point.y;
    int dl = current_start.layer - end_point.layer;
    if (dl != 0 && (dx != 0 || dy != 0)) {
        // std::cerr << "Error: Last segment has both layer change and XY movement." << std::endl;
    } else if (dx != 0 && dy != 0) {
        if (dx != dy && dx != -dy) {
            // std::cerr << "Error: Last segment is not straight or diagonal." << std::endl;
        }
    }

    segments.emplace_back(current_start, end_point);
    return segments;
}

EscapeOutput a_star_escape(
        RoutingCase& rc,
        const std::vector<PairTeam>& vpt,
        std::vector<bool> intersect_flags
) {
    std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> escape_results;
    std::unordered_map<int, std::vector<std::pair<size_t, int>>> all_fail_bumps;
    for (auto& net_name : rc.net_name_list) {
        const Net& net = rc.net_dict.at(net_name);
        for (const auto& bump_name : net.bump_names) {
            const Bump& bump = rc.bump_dict.at(bump_name);
            if (bump.layer <= bump.exit_terminal.layer) {
                for (int l = bump.layer; l <= bump.exit_terminal.layer; ++l) {
                    std::string node_key = get_node_key(bump.grid_coord_x, bump.grid_coord_y, l);
                    if (rc.occupied_set.find(node_key) == rc.occupied_set.end()) {
                        rc.occupied_set.insert(node_key);
                    }
                }
                escape_results[bump_name] = {
                        std::make_pair(
                                ResultPoint{bump.grid_coord_x, bump.grid_coord_y, bump.layer},
                                ResultPoint{bump.grid_coord_x, bump.grid_coord_y, bump.exit_terminal.layer}
                        )
                };
            } else {
                for (int l = bump.exit_terminal.layer; l <= bump.layer; ++l) {
                    std::string node_key = get_node_key(bump.grid_coord_x, bump.grid_coord_y, l);
                    if (rc.occupied_set.find(node_key) == rc.occupied_set.end()) {
                        rc.occupied_set.insert(node_key);
                    }
                }
                escape_results[bump_name] = {
                        std::make_pair(
                                ResultPoint{bump.grid_coord_x, bump.grid_coord_y, bump.layer},
                                ResultPoint{bump.grid_coord_x, bump.grid_coord_y, bump.exit_terminal.layer}
                        )
                };
            }
            // std::string bump_node_key = get_node_key(
            //         bump.grid_coord_x,
            //         bump.grid_coord_y,
            //         bump.layer
            // );
            // if (rc.occupied_set.find(bump_node_key) == rc.occupied_set.end()) {
            //     rc.occupied_set.insert(bump_node_key);
            // }
            // std::string start_node_key = get_node_key(
            //         bump.grid_coord_x,
            //         bump.grid_coord_y,
            //         bump.exit_terminal.layer
            // );
            // if (rc.occupied_set.find(start_node_key) == rc.occupied_set.end()) {
            //     rc.occupied_set.insert(start_node_key);
            // } else {
            //     if (bump.exit_terminal.layer != bump.layer) {
            //         // std::cout << "Occupied node encountered in build starts: " << start_node_key << std::endl;
            //     }
            // }
            std::string exit_node_key = get_node_key(
                    bump.exit_terminal.grid_coord_x,
                    bump.exit_terminal.grid_coord_y,
                    bump.exit_terminal.layer
            );
            if (rc.occupied_set.find(exit_node_key) == rc.occupied_set.end()) {
                rc.occupied_set.insert(exit_node_key);
            } else {
                const size_t team_count = vpt.size();
                for (size_t team_idx = 0; team_idx < team_count; ++team_idx) {
                    const PairTeam& team = vpt[team_idx];
                    for (const auto& bump_pair : team.member) {
                        if (bump_pair.first.net_name == bump.net_name) {
                            if (!intersect_flags[team_idx]) {
                                // std::cout << "Occupied node encountered in build exits: " << exit_node_key << std::endl;
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
    // int total_escaped_num = 12;
    const size_t team_count = vpt.size();
    for (size_t team_idx = 0; team_idx < team_count; ++team_idx) {
        std::vector<std::pair<size_t, int>> fail_bumps;
        const PairTeam& team = vpt[team_idx];
        // std::cout << "Processing team " << team_idx << " with " << team.cnt << " members.\n";

        if (intersect_flags[team_idx]) {
            int a_min_w = team.start_left_bound_x;
            int a_max_w = team.start_right_bound_x;
            int a_min_h = team.start_up_bound_y;
            int a_max_h = team.start_down_bound_y;

            int b_min_w = team.end_left_bound_x;
            int b_max_w = team.end_right_bound_x;
            int b_min_h = team.end_up_bound_y;
            int b_max_h = team.end_down_bound_y;

            std::vector<Point> edges = generate_cut_rect_vertices(
                    calculate_occupancy(
                            a_min_w,
                            a_max_w,
                            a_min_h,
                            a_max_h,
                            b_min_w,
                            b_max_w,
                            b_min_h,
                            b_max_h
                    )
            );

            for (size_t pair_idx = 0; pair_idx < team.member.size(); ++pair_idx) {
                const BumpPair& bump_pair = team.member[pair_idx];
                const std::string& net_name = bump_pair.first.net_name;
                if (rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).grid_coord_x != bump_pair.first.grid_coord_x ||
                    rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).grid_coord_y != bump_pair.first.grid_coord_y ||
                    rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).layer != bump_pair.first.layer) {
                    const std::string& a_bump_name = rc.net_dict.at(net_name).bump_names[1];
                    const Bump& a_bump = rc.bump_dict.at(a_bump_name);
                    AStarResult a_result = a_star_find_path_to_exit_intersect(
                            a_bump,
                            edges,
                            rc.occupied_set
                    );
                    if (a_result.path.empty() && !(a_bump.grid_coord_x == a_bump.exit_terminal.grid_coord_x && a_bump.grid_coord_y == a_bump.exit_terminal.grid_coord_y)) {
                        fail_bumps.emplace_back(pair_idx, 1);
                    }
                    std::vector<std::pair<ResultPoint, ResultPoint>> new_segments = split_to_segments(a_result.path);
                    for (AStarNode* node : a_result.nodes_to_delete) {
                        delete node;
                    }
                    a_result.nodes_to_delete.clear();
                    // // std::cout << "A* path for bump " << a_bump_name << ": ";
                    // for (auto a_node: a_result.path) {
                    //     // std::cout << "(" << a_node.x << ", " << a_node.y << ", " << a_node.layer << ") ";
                    // }
                    // // std::cout << "\n";
                    escape_results[a_bump_name].insert(
                            escape_results[a_bump_name].end(),
                            new_segments.begin(),
                            new_segments.end()
                    );
                } else {
                    const std::string& a_bump_name = rc.net_dict.at(net_name).bump_names[0];
                    const Bump& a_bump = rc.bump_dict.at(a_bump_name);
                    AStarResult a_result = a_star_find_path_to_exit_intersect(
                            a_bump,
                            edges,
                            rc.occupied_set
                    );
                    if (a_result.path.empty() && !(a_bump.grid_coord_x == a_bump.exit_terminal.grid_coord_x && a_bump.grid_coord_y == a_bump.exit_terminal.grid_coord_y)) {
                        fail_bumps.emplace_back(pair_idx, 0);
                    }
                    std::vector<std::pair<ResultPoint, ResultPoint>> new_segments = split_to_segments(a_result.path);
                    for (AStarNode* node : a_result.nodes_to_delete) {
                        delete node;
                    }
                    a_result.nodes_to_delete.clear();
                    // // std::cout << "A* path for bump " << a_bump_name << ": ";
                    // for (auto a_node: a_result.path) {
                    //     // std::cout << "(" << a_node.x << ", " << a_node.y << ", " << a_node.layer << ") ";
                    // }
                    // // std::cout << "\n";
                    escape_results[a_bump_name].insert(
                            escape_results[a_bump_name].end(),
                            new_segments.begin(),
                            new_segments.end()
                    );
                }
            }
        } else {
            int a_min_w = team.start_left_bound_x;
            int a_max_w = team.start_right_bound_x;
            int a_min_h = team.start_up_bound_y;
            int a_max_h = team.start_down_bound_y;

            int b_min_w = team.end_left_bound_x;
            int b_max_w = team.end_right_bound_x;
            int b_min_h = team.end_up_bound_y;
            int b_max_h = team.end_down_bound_y;

            for (size_t pair_idx = 0; pair_idx < team.member.size(); ++pair_idx) {
                const BumpPair& bump_pair = team.member[pair_idx];
                // total_escaped_num -= 2;
                // if (total_escaped_num <= 0) {
                //     return escape_results;
                // }
                const std::string& net_name = bump_pair.first.net_name;
                if (rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).grid_coord_x != bump_pair.first.grid_coord_x ||
                    rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).grid_coord_y != bump_pair.first.grid_coord_y ||
                    rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).layer != bump_pair.first.layer) {
                    const std::string& a_bump_name = rc.net_dict.at(net_name).bump_names[1];
                    const std::string& b_bump_name = rc.net_dict.at(net_name).bump_names[0];
                    const Bump& a_bump = rc.bump_dict.at(a_bump_name);
                    const Bump& b_bump = rc.bump_dict.at(b_bump_name);

                    AStarResult a_result = a_star_find_path_to_exit(
                            a_bump,
                            a_max_w, a_max_h,
                            a_min_w, a_min_h,
                            rc.occupied_set
                    );
                    if (a_result.path.empty() && !(a_bump.grid_coord_x == a_bump.exit_terminal.grid_coord_x && a_bump.grid_coord_y == a_bump.exit_terminal.grid_coord_y)) {
                        fail_bumps.emplace_back(pair_idx, 1);
                    }
                    std::vector<std::pair<ResultPoint, ResultPoint>> new_segments_a = split_to_segments(a_result.path);
                    for (AStarNode* node : a_result.nodes_to_delete) {
                        delete node;
                    }
                    a_result.nodes_to_delete.clear();
                    // // std::cout << "A* path for bump " << a_bump_name << ": ";
                    // for (auto a_node: a_result.path) {
                    //     // std::cout << "(" << a_node.x << ", " << a_node.y << ", " << a_node.layer << ") ";
                    // }
                    // // std::cout << "\n";
                    escape_results[a_bump_name].insert(
                            escape_results[a_bump_name].end(),
                            new_segments_a.begin(),
                            new_segments_a.end()
                    );

                    AStarResult b_result = a_star_find_path_to_exit(
                            b_bump,
                            b_max_w, b_max_h,
                            b_min_w, b_min_h,
                            rc.occupied_set
                    );
                    if (b_result.path.empty() && !(b_bump.grid_coord_x == b_bump.exit_terminal.grid_coord_x && b_bump.grid_coord_y == b_bump.exit_terminal.grid_coord_y)) {
                        fail_bumps.emplace_back(pair_idx, 0);
                    }
                    std::vector<std::pair<ResultPoint, ResultPoint>> new_segments_b = split_to_segments(b_result.path);
                    for (AStarNode* node : b_result.nodes_to_delete) {
                        delete node;
                    }
                    b_result.nodes_to_delete.clear();
                    // // std::cout << "A* path for bump " << b_bump_name << ": ";
                    // for (auto b_node: b_result.path) {
                    //     // std::cout << "(" << b_node.x << ", " << b_node.y << ", " << b_node.layer << ") ";
                    // }
                    // // std::cout << "\n";
                    escape_results[b_bump_name].insert(
                            escape_results[b_bump_name].end(),
                            new_segments_b.begin(),
                            new_segments_b.end()
                    );
                } else {
                    const std::string& a_bump_name = rc.net_dict.at(net_name).bump_names[0];
                    const std::string& b_bump_name = rc.net_dict.at(net_name).bump_names[1];
                    const Bump& a_bump = rc.bump_dict.at(a_bump_name);
                    const Bump& b_bump = rc.bump_dict.at(b_bump_name);

                    AStarResult a_result = a_star_find_path_to_exit(
                            a_bump,
                            a_max_w, a_max_h,
                            a_min_w, a_min_h,
                            rc.occupied_set
                    );
                    if (a_result.path.empty() && !(a_bump.grid_coord_x == a_bump.exit_terminal.grid_coord_x && a_bump.grid_coord_y == a_bump.exit_terminal.grid_coord_y)) {
                        fail_bumps.emplace_back(pair_idx, 0);
                    }
                    std::vector<std::pair<ResultPoint, ResultPoint>> new_segments_a = split_to_segments(a_result.path);
                    for (AStarNode* node : a_result.nodes_to_delete) {
                        delete node;
                    }
                    a_result.nodes_to_delete.clear();
                    // // std::cout << "A* path for bump " << a_bump_name << ": ";
                    // for (auto a_node: a_result.path) {
                    //     // std::cout << "(" << a_node.x << ", " << a_node.y << ", " << a_node.layer << ") ";
                    // }
                    // // std::cout << "\n";
                    escape_results[a_bump_name].insert(
                            escape_results[a_bump_name].end(),
                            new_segments_a.begin(),
                            new_segments_a.end()
                    );

                    AStarResult b_result = a_star_find_path_to_exit(
                            b_bump,
                            b_max_w, b_max_h,
                            b_min_w, b_min_h,
                            rc.occupied_set
                    );
                    if (b_result.path.empty() && !(b_bump.grid_coord_x == b_bump.exit_terminal.grid_coord_x && b_bump.grid_coord_y == b_bump.exit_terminal.grid_coord_y)) {
                        fail_bumps.emplace_back(pair_idx, 1);
                    }
                    std::vector<std::pair<ResultPoint, ResultPoint>> new_segments_b = split_to_segments(b_result.path);
                    for (AStarNode* node : b_result.nodes_to_delete) {
                        delete node;
                    }
                    b_result.nodes_to_delete.clear();
                    // // std::cout << "A* path for bump " << b_bump_name << ": ";
                    // for (auto b_node: b_result.path) {
                    //     // std::cout << "(" << b_node.x << ", " << b_node.y << ", " << b_node.layer << ") ";
                    // }
                    // // std::cout << "\n";
                    escape_results[b_bump_name].insert(
                            escape_results[b_bump_name].end(),
                            new_segments_b.begin(),
                            new_segments_b.end()
                    );
                }
            }
        }
        for (const auto& fail_entry : fail_bumps) {
            size_t pair_idx = fail_entry.first;
            int fail_bump_idx = fail_entry.second;
            const BumpPair& bump_pair = team.member[pair_idx];
            const std::string& net_name = bump_pair.first.net_name;
            const std::string& fail_bump_name = rc.net_dict.at(net_name).bump_names[fail_bump_idx];
            // const Bump& fail_bump = rc.bump_dict.at(fail_bump_name);
            // if (fail_bump.layer < fail_bump.exit_terminal.layer) {
            //     for (int l = fail_bump.layer+1; l <= fail_bump.exit_terminal.layer; ++l) {
            //         rc.occupied_set.erase(get_node_key(fail_bump.grid_coord_x, fail_bump.grid_coord_y, l));
            //     }
            //     escape_results[fail_bump_name] = {};
            // } else if (fail_bump.layer > fail_bump.exit_terminal.layer) {
            //     for (int l = fail_bump.exit_terminal.layer; l < fail_bump.layer; ++l) {
            //         rc.occupied_set.erase(get_node_key(fail_bump.grid_coord_x, fail_bump.grid_coord_y, l));
            //     }
            //     escape_results[fail_bump_name] = {};
            // }
            escape_results[fail_bump_name] = {};
        }
        all_fail_bumps[team_idx] = fail_bumps;
    }
    // for (const auto& entry : escape_results) {
    //     const std::string& bump_name = entry.first;
    //     const std::vector<std::pair<ResultPoint, ResultPoint>>& segments = entry.second;
    //     // std::cout << "Bump: " << bump_name << "\n";
    //     for (const auto& segment : segments) {
    //         const ResultPoint& start = segment.first;
    //         const ResultPoint& end = segment.second;
    //         // std::cout << "  Segment from (" << start.x << ", " << start.y << ", " << start.layer << ") to ("
    //                   << end.x << ", " << end.y << ", " << end.layer << ")\n";
    //     }
    // }
    EscapeOutput output;
    output.escape_results = escape_results;
    output.all_fail_bumps = all_fail_bumps;
    return output;
}

// ====================================Multi Thread====================================

// 函数参数新增std::mutex& rc_mutex
std::vector<AStarNode> get_neighbors_lock(
        const AStarNode& current,
        int max_width, int max_height,
        int min_width, int min_height,
        const std::unordered_set<std::string>& occupied_set,
        int end_x, int end_y, int end_layer,
        std::mutex& rc_mutex  // 新增：保护occupied_set的锁
) {
    std::vector<AStarNode> neighbors;
    std::vector<std::pair<int, int>> dirs = {{0,1},{0,-1},{1,0},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}};

    for (const auto& dir : dirs) {
        int dx = dir.first, dy = dir.second;
        int new_x = current.x + dx, new_y = current.y + dy;

        if (new_x >= min_width && new_x <= max_width && new_y >= min_height && new_y <= max_height) {
            std::string coord = get_node_key(new_x, new_y, end_layer);
            // 读操作加锁
            bool coord_occupied = false;
            {
                std::lock_guard<std::mutex> lock(rc_mutex);
                coord_occupied = (occupied_set.find(coord) != occupied_set.end());
            }
            if (coord_occupied && !(new_x == end_x && new_y == end_y)) {
                continue;
            }

            if (dx != 0 && dy != 0) {
                if (current.parent != nullptr && ((new_x == current.parent->x && current.y == current.parent->y) || (new_y == current.parent->y && current.x == current.parent->x))) {
                    continue;
                }
                float mid_x = current.x + dx * 0.5f, mid_y = current.y + dy * 0.5f;
                std::string key = get_node_key(mid_x, mid_y, end_layer);
                // 读操作加锁
                bool key_occupied = false;
                {
                    std::lock_guard<std::mutex> lock(rc_mutex);
                    key_occupied = (occupied_set.find(key) != occupied_set.end());
                }
                if (key_occupied) {
                    continue;
                }
            } else if (dx != 0) {
                if (current.parent != nullptr && new_x == current.parent->x) {
                    continue;
                }
            } else if (dy != 0) {
                if (current.parent != nullptr && new_y == current.parent->y) {
                    continue;
                }
            }
            neighbors.emplace_back(new_x, new_y, end_layer);
        }
    }
    return neighbors;
}

// 函数参数新增std::mutex& rc_mutex
std::vector<AStarNode> get_neighbors_intersect_lock(
        const AStarNode& current,
        const std::vector<Point>& edges,
        const std::unordered_set<std::string>& occupied_set,
        int end_x, int end_y, int end_layer,
        std::mutex& rc_mutex  // 新增：保护occupied_set的锁
) {
    std::vector<AStarNode> neighbors;
    std::vector<std::pair<int, int>> dirs = {{0,1},{0,-1},{1,0},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}};

    for (const auto& dir : dirs) {
        int dx = dir.first, dy = dir.second;
        int new_x = current.x + dx, new_y = current.y + dy;

        if (is_point_in_convex_polygon(new_x, new_y, edges)) {
            std::string coord = get_node_key(new_x, new_y, end_layer);
            // 读操作加锁
            bool coord_occupied = false;
            {
                std::lock_guard<std::mutex> lock(rc_mutex);
                coord_occupied = (occupied_set.find(coord) != occupied_set.end());
            }
            if (coord_occupied && !(new_x == end_x && new_y == end_y)) {
                continue;
            }

            if (dx != 0 && dy != 0) {
                if (current.parent != nullptr && ((new_x == current.parent->x && current.y == current.parent->y) || (new_y == current.parent->y && current.x == current.parent->x))) {
                    continue;
                }
                float mid_x = current.x + dx * 0.5f, mid_y = current.y + dy * 0.5f;
                std::string key = get_node_key(mid_x, mid_y, end_layer);
                // 读操作加锁
                bool key_occupied = false;
                {
                    std::lock_guard<std::mutex> lock(rc_mutex);
                    key_occupied = (occupied_set.find(key) != occupied_set.end());
                }
                if (key_occupied) {
                    continue;
                }
            } else if (dx != 0) {
                if (current.parent != nullptr && new_x == current.parent->x) {
                    continue;
                }
            } else if (dy != 0) {
                if (current.parent != nullptr && new_y == current.parent->y) {
                    continue;
                }
            }
            neighbors.emplace_back(new_x, new_y, end_layer);
        }
    }
    return neighbors;
}

// 函数参数新增std::mutex& rc_mutex
std::vector<AStarNode> reconstruct_path_lock(
        AStarNode* end_node,
        std::unordered_set<std::string>& occupied_set,
        std::mutex& rc_mutex  // 新增：保护occupied_set的锁
) {
    std::vector<AStarNode> path;
    AStarNode* current = end_node;
    AStarNode* last = end_node;
    while (current != nullptr) {
        path.push_back(*current);
        // 写操作加锁
        {
            std::lock_guard<std::mutex> lock(rc_mutex);
            occupied_set.insert(get_node_key(current->x, current->y, current->layer));
        }
        if (last->x != current->x && last->y != current->y) {
            float mid_x = (last->x + current->x) * 0.5f, mid_y = (last->y + current->y) * 0.5f;
            // 写操作加锁
            {
                std::lock_guard<std::mutex> lock(rc_mutex);
                occupied_set.insert(get_node_key(mid_x, mid_y, current->layer));
            }
        }
        last = current;
        current = current->parent;
    }
    return path;
}

AStarResult a_star_find_path_to_exit_lock(
        const Bump& bump,
        int max_width,
        int max_height,
        int min_width,
        int min_height,
        std::unordered_set<std::string>& occupied_set,
        std::mutex& rc_mutex
) {
    const ExitTerminal& target_exit = bump.exit_terminal;
    int end_x = target_exit.grid_coord_x;
    int end_y = target_exit.grid_coord_y;
    int end_layer = target_exit.layer;
    // // std::cout << "A* routing to exit for bump " << bump.net_name << " from (" << bump.grid_coord_x << ", " << bump.grid_coord_y << ", " << bump.layer << ") to exit (" << end_x << ", " << end_y << ", " << end_layer << ")\n";
    // // std::cout << "width range: [" << min_width << ", " << max_width << "], height range: [" << min_height << ", " << max_height << "]\n";

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, int> best_g;
    std::unordered_set<AStarNode*> all_allocated_nodes;
    AStarResult routing_result;

    AStarNode start_node(bump.grid_coord_x, bump.grid_coord_y, end_layer);
    start_node.h = grid_distance(
            start_node.x, start_node.y,
            end_x, end_y
    );
    start_node.f = start_node.g + start_node.h;
    open_list.push(start_node);
    best_g[get_coord_node_key(start_node.x, start_node.y, start_node.layer)] = 0;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        // // std::cout << "current g: " << current.g << "\n";
        open_list.pop();
        std::string current_key = get_coord_node_key(current.x, current.y, current.layer);

        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            // // std::cout << "Path found with length: " << current.g << "\n";
            routing_result.path = reconstruct_path_lock(&current, occupied_set, rc_mutex);
            routing_result.nodes_to_delete = std::move(all_allocated_nodes);
            return routing_result;
        }

        if (closed_set.count(current_key) && best_g[current_key] < current.g) {
            continue;
        }
        closed_set.insert(current_key);

        std::vector<AStarNode> neighbors = get_neighbors_lock(current, max_width, max_height, min_width, min_height, occupied_set, end_x, end_y, end_layer, rc_mutex);
        for (AStarNode& neighbor : neighbors) {
            std::string neighbor_key = get_coord_node_key(neighbor.x, neighbor.y, neighbor.layer);
            if (closed_set.count(neighbor_key)) {
                continue;
            }

            int tentative_g = current.g + 1;
            if (!best_g.count(neighbor_key) || tentative_g < best_g[neighbor_key]) {
                neighbor.parent = new AStarNode(current);
                all_allocated_nodes.insert(neighbor.parent);
                neighbor.g = tentative_g;
                neighbor.h = grid_distance(
                        neighbor.x, neighbor.y,
                        end_x, end_y
                );
                neighbor.f = neighbor.g + neighbor.h;

                open_list.push(neighbor);
                best_g[neighbor_key] = tentative_g;
            }
        }
    }

    // // std::cout << "No path found for bump " << bump.net_name << " from (" << bump.grid_coord_x << ", " << bump.grid_coord_y << ", " << bump.layer << ") to exit (" << end_x << ", " << end_y << ", " << end_layer << ")\n";
    // // std::cout << "    routing range: width range: [" << min_width << ", " << max_width << "], height range: [" << min_height << ", " << max_height << "]\n";
    routing_result.path = {};
    routing_result.nodes_to_delete = std::move(all_allocated_nodes);
    return routing_result;
}

AStarResult a_star_find_path_to_exit_intersect_lock(
        const Bump& bump,
        const std::vector<Point>& edges,
        std::unordered_set<std::string>& occupied_set,
        std::mutex& rc_mutex
) {
    const ExitTerminal& target_exit = bump.exit_terminal;
    int end_x = target_exit.grid_coord_x;
    int end_y = target_exit.grid_coord_y;
    int end_layer = target_exit.layer;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, int> best_g;
    std::unordered_set<AStarNode*> all_allocated_nodes;
    AStarResult routing_result;

    AStarNode start_node(bump.grid_coord_x, bump.grid_coord_y, end_layer);
    start_node.h = grid_distance(
            start_node.x, start_node.y,
            end_x, end_y
    );
    start_node.f = start_node.g + start_node.h;
    open_list.push(start_node);
    best_g[get_coord_node_key(start_node.x, start_node.y, start_node.layer)] = 0;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        // // std::cout << "current g: " << current.g << "\n";
        open_list.pop();
        std::string current_key = get_coord_node_key(current.x, current.y, current.layer);

        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            // // std::cout << "Path found with length: " << current.g << "\n";
            routing_result.path = reconstruct_path_lock(&current, occupied_set, rc_mutex);
            routing_result.nodes_to_delete = std::move(all_allocated_nodes);
            return routing_result;
        }

        if (closed_set.count(current_key) && best_g[current_key] < current.g) {
            continue;
        }
        closed_set.insert(current_key);

        std::vector<AStarNode> neighbors = get_neighbors_intersect_lock(current, edges, occupied_set, end_x, end_y, end_layer, rc_mutex);
        for (AStarNode& neighbor : neighbors) {
            std::string neighbor_key = get_coord_node_key(neighbor.x, neighbor.y, neighbor.layer);
            if (closed_set.count(neighbor_key)) {
                continue;
            }

            int tentative_g = current.g + 1;
            if (!best_g.count(neighbor_key) || tentative_g < best_g[neighbor_key]) {
                neighbor.parent = new AStarNode(current);
                all_allocated_nodes.insert(neighbor.parent);
                neighbor.g = tentative_g;
                neighbor.h = grid_distance(
                        neighbor.x, neighbor.y,
                        end_x, end_y
                );
                neighbor.f = neighbor.g + neighbor.h;

                open_list.push(neighbor);
                best_g[neighbor_key] = tentative_g;
            }
        }
    }

    // // std::cout << "No path found for bump " << bump.net_name << " from (" << bump.grid_coord_x << ", " << bump.grid_coord_y << ", " << bump.layer << ") to exit (" << end_x << ", " << end_y << ", " << end_layer << ")\n";
    // // std::cout << "    routing range:\n";
    // for (const auto& edge : edges) {
    //     // std::cout << "        (" << edge.first << ", " << edge.second << "); ";
    // }
    // // std::cout << "\n";
    routing_result.path = {};
    routing_result.nodes_to_delete = std::move(all_allocated_nodes);
    return routing_result;
}

void a_star_escape_per_team(
        size_t team_idx,
        const PairTeam& team,
        bool intersect_flag,
        RoutingCase& rc,
        std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> &escape_results,
        std::unordered_map<int, std::vector<std::pair<size_t, int>>> &all_fail_bumps,
        std::mutex& rc_mutex,          // 新增：保护rc.occupied_set的锁
        std::mutex& escape_mutex,      // 新增：保护escape_results的锁
        std::mutex& fail_bumps_mutex   // 新增：保护all_fail_bumps的锁
)
{
    std::vector<std::pair<size_t, int>> fail_bumps;
    // std::cout << "Processing team " << team_idx << " with " << team.cnt << " members.\n";

    if (intersect_flag) {
        int a_min_w = team.start_left_bound_x;
        int a_max_w = team.start_right_bound_x;
        int a_min_h = team.start_up_bound_y;
        int a_max_h = team.start_down_bound_y;

        int b_min_w = team.end_left_bound_x;
        int b_max_w = team.end_right_bound_x;
        int b_min_h = team.end_up_bound_y;
        int b_max_h = team.end_down_bound_y;

        std::vector<Point> edges = generate_cut_rect_vertices(
                calculate_occupancy(
                        a_min_w,
                        a_max_w,
                        a_min_h,
                        a_max_h,
                        b_min_w,
                        b_max_w,
                        b_min_h,
                        b_max_h
                )
        );

        for (size_t pair_idx = 0; pair_idx < team.member.size(); ++pair_idx) {
            const BumpPair& bump_pair = team.member[pair_idx];
            const std::string& net_name = bump_pair.first.net_name;
            if (rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).grid_coord_x != bump_pair.first.grid_coord_x ||
                rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).grid_coord_y != bump_pair.first.grid_coord_y ||
                rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).layer != bump_pair.first.layer) {
                const std::string& a_bump_name = rc.net_dict.at(net_name).bump_names[1];
                const Bump& a_bump = rc.bump_dict.at(a_bump_name);
                AStarResult a_result = a_star_find_path_to_exit_intersect_lock(
                        a_bump,
                        edges,
                        rc.occupied_set,
                        rc_mutex
                );
                if (a_result.path.empty() && !(a_bump.grid_coord_x == a_bump.exit_terminal.grid_coord_x && a_bump.grid_coord_y == a_bump.exit_terminal.grid_coord_y)) {
                    fail_bumps.emplace_back(pair_idx, 1);
                }
                std::vector<std::pair<ResultPoint, ResultPoint>> new_segments = split_to_segments(a_result.path);
                for (AStarNode* node : a_result.nodes_to_delete) {
                    delete node;
                }
                a_result.nodes_to_delete.clear();
                // 加锁保护escape_results写入
                {
                    std::lock_guard<std::mutex> lock(escape_mutex);
                    escape_results[a_bump_name].insert(
                            escape_results[a_bump_name].end(),
                            new_segments.begin(),
                            new_segments.end()
                    );
                }
            } else {
                const std::string& a_bump_name = rc.net_dict.at(net_name).bump_names[0];
                const Bump& a_bump = rc.bump_dict.at(a_bump_name);
                AStarResult a_result = a_star_find_path_to_exit_intersect_lock(
                        a_bump,
                        edges,
                        rc.occupied_set,
                        rc_mutex
                );
                if (a_result.path.empty() && !(a_bump.grid_coord_x == a_bump.exit_terminal.grid_coord_x && a_bump.grid_coord_y == a_bump.exit_terminal.grid_coord_y)) {
                    fail_bumps.emplace_back(pair_idx, 0);
                }
                std::vector<std::pair<ResultPoint, ResultPoint>> new_segments = split_to_segments(a_result.path);
                for (AStarNode* node : a_result.nodes_to_delete) {
                    delete node;
                }
                a_result.nodes_to_delete.clear();
                // 加锁保护escape_results写入
                {
                    std::lock_guard<std::mutex> lock(escape_mutex);
                    escape_results[a_bump_name].insert(
                            escape_results[a_bump_name].end(),
                            new_segments.begin(),
                            new_segments.end()
                    );
                }
            }
        }
    } else {
        int a_min_w = team.start_left_bound_x;
        int a_max_w = team.start_right_bound_x;
        int a_min_h = team.start_up_bound_y;
        int a_max_h = team.start_down_bound_y;

        int b_min_w = team.end_left_bound_x;
        int b_max_w = team.end_right_bound_x;
        int b_min_h = team.end_up_bound_y;
        int b_max_h = team.end_down_bound_y;

        for (size_t pair_idx = 0; pair_idx < team.member.size(); ++pair_idx) {
            const BumpPair& bump_pair = team.member[pair_idx];
            const std::string& net_name = bump_pair.first.net_name;
            if (rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).grid_coord_x != bump_pair.first.grid_coord_x ||
                rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).grid_coord_y != bump_pair.first.grid_coord_y ||
                rc.bump_dict.at(rc.net_dict.at(net_name).bump_names[0]).layer != bump_pair.first.layer) {
                const std::string& a_bump_name = rc.net_dict.at(net_name).bump_names[1];
                const std::string& b_bump_name = rc.net_dict.at(net_name).bump_names[0];
                const Bump& a_bump = rc.bump_dict.at(a_bump_name);
                const Bump& b_bump = rc.bump_dict.at(b_bump_name);

                AStarResult a_result = a_star_find_path_to_exit_lock(
                        a_bump,
                        a_max_w, a_max_h,
                        a_min_w, a_min_h,
                        rc.occupied_set,
                        rc_mutex
                );
                if (a_result.path.empty() && !(a_bump.grid_coord_x == a_bump.exit_terminal.grid_coord_x && a_bump.grid_coord_y == a_bump.exit_terminal.grid_coord_y)) {
                    fail_bumps.emplace_back(pair_idx, 1);
                }
                std::vector<std::pair<ResultPoint, ResultPoint>> new_segments_a = split_to_segments(a_result.path);
                for (AStarNode* node : a_result.nodes_to_delete) {
                    delete node;
                }
                a_result.nodes_to_delete.clear();
                // 加锁保护escape_results写入
                {
                    std::lock_guard<std::mutex> lock(escape_mutex);
                    escape_results[a_bump_name].insert(
                            escape_results[a_bump_name].end(),
                            new_segments_a.begin(),
                            new_segments_a.end()
                    );
                }

                AStarResult b_result = a_star_find_path_to_exit_lock(
                        b_bump,
                        b_max_w, b_max_h,
                        b_min_w, b_min_h,
                        rc.occupied_set,
                        rc_mutex
                );
                if (b_result.path.empty() && !(b_bump.grid_coord_x == b_bump.exit_terminal.grid_coord_x && b_bump.grid_coord_y == b_bump.exit_terminal.grid_coord_y)) {
                    fail_bumps.emplace_back(pair_idx, 0);
                }
                std::vector<std::pair<ResultPoint, ResultPoint>> new_segments_b = split_to_segments(b_result.path);
                for (AStarNode* node : b_result.nodes_to_delete) {
                    delete node;
                }
                b_result.nodes_to_delete.clear();
                // 加锁保护escape_results写入
                {
                    std::lock_guard<std::mutex> lock(escape_mutex);
                    escape_results[b_bump_name].insert(
                            escape_results[b_bump_name].end(),
                            new_segments_b.begin(),
                            new_segments_b.end()
                    );
                }
            } else {
                const std::string& a_bump_name = rc.net_dict.at(net_name).bump_names[0];
                const std::string& b_bump_name = rc.net_dict.at(net_name).bump_names[1];
                const Bump& a_bump = rc.bump_dict.at(a_bump_name);
                const Bump& b_bump = rc.bump_dict.at(b_bump_name);

                AStarResult a_result = a_star_find_path_to_exit_lock(
                        a_bump,
                        a_max_w, a_max_h,
                        a_min_w, a_min_h,
                        rc.occupied_set,
                        rc_mutex
                );
                if (a_result.path.empty() && !(a_bump.grid_coord_x == a_bump.exit_terminal.grid_coord_x && a_bump.grid_coord_y == a_bump.exit_terminal.grid_coord_y)) {
                    fail_bumps.emplace_back(pair_idx, 0);
                }
                std::vector<std::pair<ResultPoint, ResultPoint>> new_segments_a = split_to_segments(a_result.path);
                for (AStarNode* node : a_result.nodes_to_delete) {
                    delete node;
                }
                a_result.nodes_to_delete.clear();
                // 加锁保护escape_results写入
                {
                    std::lock_guard<std::mutex> lock(escape_mutex);
                    escape_results[a_bump_name].insert(
                            escape_results[a_bump_name].end(),
                            new_segments_a.begin(),
                            new_segments_a.end()
                    );
                }

                AStarResult b_result = a_star_find_path_to_exit_lock(
                        b_bump,
                        b_max_w, b_max_h,
                        b_min_w, b_min_h,
                        rc.occupied_set,
                        rc_mutex
                );
                if (b_result.path.empty() && !(b_bump.grid_coord_x == b_bump.exit_terminal.grid_coord_x && b_bump.grid_coord_y == b_bump.exit_terminal.grid_coord_y)) {
                    fail_bumps.emplace_back(pair_idx, 1);
                }
                std::vector<std::pair<ResultPoint, ResultPoint>> new_segments_b = split_to_segments(b_result.path);
                for (AStarNode* node : b_result.nodes_to_delete) {
                    delete node;
                }
                b_result.nodes_to_delete.clear();
                // 加锁保护escape_results写入
                {
                    std::lock_guard<std::mutex> lock(escape_mutex);
                    escape_results[b_bump_name].insert(
                            escape_results[b_bump_name].end(),
                            new_segments_b.begin(),
                            new_segments_b.end()
                    );
                }
            }
        }
    }

    // 处理失败bumps：修改rc.occupied_set和escape_results
    for (const auto& fail_entry : fail_bumps) {
        size_t pair_idx = fail_entry.first;
        int fail_bump_idx = fail_entry.second;
        const BumpPair& bump_pair = team.member[pair_idx];
        const std::string& net_name = bump_pair.first.net_name;
        const std::string& fail_bump_name = rc.net_dict.at(net_name).bump_names[fail_bump_idx];
        // const Bump& fail_bump = rc.bump_dict.at(fail_bump_name);

        // if (fail_bump.layer < fail_bump.exit_terminal.layer) {
        //     // 加锁保护rc.occupied_set的erase操作
        //     {
        //         std::lock_guard<std::mutex> rc_lock(rc_mutex);
        //         for (int l = fail_bump.layer+1; l <= fail_bump.exit_terminal.layer; ++l) {
        //             rc.occupied_set.erase(get_node_key(fail_bump.grid_coord_x, fail_bump.grid_coord_y, l));
        //         }
        //     }
        //     // 加锁清空escape_results对应项
        //     {
        //         std::lock_guard<std::mutex> escape_lock(escape_mutex);
        //         escape_results[fail_bump_name] = {};
        //     }
        // } else if (fail_bump.layer > fail_bump.exit_terminal.layer) {
        //     // 加锁保护rc.occupied_set的erase操作
        //     {
        //         std::lock_guard<std::mutex> rc_lock(rc_mutex);
        //         for (int l = fail_bump.exit_terminal.layer; l < fail_bump.layer; ++l) {
        //             rc.occupied_set.erase(get_node_key(fail_bump.grid_coord_x, fail_bump.grid_coord_y, l));
        //         }
        //     }
        //     // 加锁清空escape_results对应项
        //     {
        //         std::lock_guard<std::mutex> escape_lock(escape_mutex);
        //         escape_results[fail_bump_name] = {};
        //     }
        // }
        {
            std::lock_guard<std::mutex> escape_lock(escape_mutex);
            escape_results[fail_bump_name] = {};
        }
    }

    // 加锁保护all_fail_bumps的赋值操作
    {
        std::lock_guard<std::mutex> fail_lock(fail_bumps_mutex);
        all_fail_bumps[team_idx] = fail_bumps;
    }
}

EscapeOutput a_star_escape_multithread(
        RoutingCase& rc,
        const std::vector<PairTeam>& vpt,
        std::vector<bool> intersect_flags
) {
    std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> escape_results;
    std::unordered_map<int, std::vector<std::pair<size_t, int>>> all_fail_bumps;
    std::vector<std::string> layer_fail_bumps;

    // 单线程初始化：无竞争，无需加锁
    for (auto& net_name : rc.net_name_list) {
        const Net& net = rc.net_dict.at(net_name);
        for (const auto& bump_name : net.bump_names) {
            const Bump& bump = rc.bump_dict.at(bump_name);
            if (bump.layer <= bump.exit_terminal.layer) {
                for (int l = bump.layer; l <= bump.exit_terminal.layer; ++l) {
                    std::string node_key = get_node_key(bump.grid_coord_x, bump.grid_coord_y, l);
                    if (rc.occupied_set.find(node_key) == rc.occupied_set.end()) {
                        rc.occupied_set.insert(node_key);
                    } else {
                        // std::cout << "Occupied node encountered during escape initialization: " << node_key << std::endl;
                    }
                }
                escape_results[bump_name] = {
                        std::make_pair(
                                ResultPoint{bump.grid_coord_x, bump.grid_coord_y, bump.layer},
                                ResultPoint{bump.grid_coord_x, bump.grid_coord_y, bump.exit_terminal.layer}
                        )
                };
            } else {
                for (int l = bump.exit_terminal.layer; l <= bump.layer; ++l) {
                    std::string node_key = get_node_key(bump.grid_coord_x, bump.grid_coord_y, l);
                    if (rc.occupied_set.find(node_key) == rc.occupied_set.end()) {
                        rc.occupied_set.insert(node_key);
                    } else {
                        // std::cout << "Occupied node encountered during escape initialization: " << node_key << std::endl;
                    }
                }
                escape_results[bump_name] = {
                        std::make_pair(
                                ResultPoint{bump.grid_coord_x, bump.grid_coord_y, bump.layer},
                                ResultPoint{bump.grid_coord_x, bump.grid_coord_y, bump.exit_terminal.layer}
                        )
                };
            }
            // std::string bump_node_key = get_node_key(
            //         bump.grid_coord_x,
            //         bump.grid_coord_y,
            //         bump.layer
            // );
            // if (rc.occupied_set.find(bump_node_key) == rc.occupied_set.end()) {
            //     rc.occupied_set.insert(bump_node_key);
            // }
            // std::string start_node_key = get_node_key(
            //         bump.grid_coord_x,
            //         bump.grid_coord_y,
            //         bump.exit_terminal.layer
            // );
            // if (rc.occupied_set.find(start_node_key) == rc.occupied_set.end()) {
            //     rc.occupied_set.insert(start_node_key);
            // } else {
            //     if (bump.exit_terminal.layer != bump.layer) {
            //         // std::cout << "Occupied node encountered in build starts: " << start_node_key << std::endl;
            //     }
            // }
            std::string exit_node_key = get_node_key(
                    bump.exit_terminal.grid_coord_x,
                    bump.exit_terminal.grid_coord_y,
                    bump.exit_terminal.layer
            );
            if (rc.occupied_set.find(exit_node_key) == rc.occupied_set.end()) {
                rc.occupied_set.insert(exit_node_key);
            } else {
                const size_t team_count = vpt.size();
                for (size_t team_idx = 0; team_idx < team_count; ++team_idx) {
                    const PairTeam& team = vpt[team_idx];
                    for (const auto& bump_pair : team.member) {
                        if (bump_pair.first.net_name == bump.net_name) {
                            if (!intersect_flags[team_idx]) {
                                // std::cout << "Occupied node encountered in build exits: " << exit_node_key << std::endl;
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    const size_t team_count = vpt.size();

    // 初始化互斥锁：分别保护不同共享资源
    std::mutex rc_mutex;          // 保护rc.occupied_set
    std::mutex escape_mutex;      // 保护escape_results
    std::mutex fail_bumps_mutex;  // 保护all_fail_bumps

    // 1. 设定线程数（对应原ThreadPool）
    omp_set_num_threads(8);

    // 2. 提前声明所有需要私有化的变量（仅声明，不赋值）
    size_t team_idx;                // 仅声明，循环内赋值
    const PairTeam* team;           // 指针类型，避免拷贝+适配private
    bool intersect_flag;            // 仅声明，循环内赋值

    // 3. OpenMP并行循环（修正后，无重定义）
    #pragma omp parallel for \
        private(team_idx, team, intersect_flag) \
        shared(vpt, intersect_flags, rc, escape_results, all_fail_bumps, rc_mutex, escape_mutex, fail_bumps_mutex)
    // 循环内不再重新定义变量，直接给提前声明的变量赋值
    for (team_idx = 0; team_idx < team_count; ++team_idx) {
        // 给提前声明的变量赋值（无重定义，类型匹配）
        team = &vpt[team_idx];              // 指针指向vpt的元素，避免拷贝
        intersect_flag = intersect_flags[team_idx];

        // 调用函数：team指针解引用为const PairTeam&，匹配函数参数类型
        a_star_escape_per_team(
                team_idx,
                *team,                       // 解引用指针，还原为const PairTeam&
                intersect_flag,
                rc,
                escape_results,
                all_fail_bumps,
                rc_mutex,
                escape_mutex,
                fail_bumps_mutex
        );
    }

    EscapeOutput output;
    output.escape_results = escape_results;
    output.all_fail_bumps = all_fail_bumps;
    return output;
}