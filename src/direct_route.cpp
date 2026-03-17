#include "direct_route.h"

typedef std::pair<std::vector<ExitPos>, std::vector<ExitPos>> ExitBumpPos;

AStarNodeReroute::AStarNodeReroute(int x_, int y_, int layer_)
    : x(x_), y(y_), layer(layer_), g(0), h(0), f(0), parent(nullptr) {}


bool AStarNodeReroute::operator>(const AStarNodeReroute& other) const {
    return f > other.f;
}


std::string get_coord_node_key_reroute(int x, int y, int layer) {
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(layer);
}


std::string get_node_key_reroute(float x, float y, int layer) {
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(layer);
}


int grid_distance_reroute(int x1, int y1, int l1, int x2, int y2, int l2) {
    return std::max(abs(x1 - x2), abs(y1 - y2)) + abs(l1 - l2);
}

std::vector<AStarNodeReroute> get_neighbors_reroute(const AStarNodeReroute& current, int max_width, int max_height, int layer_count, const std::unordered_set<std::string>& occupied_set, int end_x, int end_y, int end_layer) {
    std::vector<AStarNodeReroute> neighbors;
    std::vector<std::tuple<int, int, int>> dirs = {
        std::make_tuple(0, 1, 0),
        std::make_tuple(0, -1, 0),
        std::make_tuple(1, 0, 0),
        std::make_tuple(-1, 0, 0),
        std::make_tuple(1, 1, 0),
        std::make_tuple(1, -1, 0),
        std::make_tuple(-1, 1, 0),
        std::make_tuple(-1, -1, 0),
        std::make_tuple(0, 0, 1),
        std::make_tuple(0, 0, -1)
    };

    for (const auto& dir : dirs) {
        int dx = std::get<0>(dir);
        int dy = std::get<1>(dir);
        int dl = std::get<2>(dir);

        int new_x = current.x + dx;
        int new_y = current.y + dy;
        int new_layer = current.layer + dl;

        if (new_x >= 0 && new_x < max_width && new_y >= 0 && new_y < max_height && new_layer >= 0 && new_layer < layer_count) {
            std::string coord = get_node_key_reroute(new_x, new_y, new_layer);
            if (occupied_set.find(coord) != occupied_set.end()) {
                if (!(new_x == end_x && new_y == end_y && new_layer == end_layer)) {
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
                std::string key = get_node_key_reroute(mid_x, mid_y, new_layer);
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
            neighbors.emplace_back(new_x, new_y, new_layer);
        }
    }
    return neighbors;
}


std::vector<AStarNodeReroute> reconstruct_path_reroute(AStarNodeReroute* end_node, std::unordered_set<std::string>& occupied_set) {
    std::vector<AStarNodeReroute> path;
    AStarNodeReroute* current = end_node;
    AStarNodeReroute* last = end_node;
    while (current != nullptr) {
        path.push_back(*current);
        std::string last_node_key = get_node_key_reroute(current->x, current->y, current->layer);
        if (occupied_set.find(last_node_key) == occupied_set.end()) {
            occupied_set.insert(last_node_key);
        }
        if ((last->x != current->x) && (last->y != current->y)) {
            float mid_x = static_cast<float>(last->x + current->x) * 0.5f;
            float mid_y = static_cast<float>(last->y + current->y) * 0.5f;
            std::string mid_node_key = get_node_key_reroute(mid_x, mid_y, current->layer);
            if (occupied_set.find(mid_node_key) == occupied_set.end()) {
                occupied_set.insert(mid_node_key);
            } else {
                // std::cout << "Occupied node encountered in reroute: " << mid_node_key << std::endl;
            }
        }
        last = current;
        current = current->parent;
    }
    return path;
}


AStarResultReroute a_star_find_path_reroute(const BumpPair& bp, int max_width, int max_height, int layer_count, std::unordered_set<std::string>& occupied_set) {
    const ExitTerminal& start_exit = bp.first.exit_terminal;
    const ExitTerminal& target_exit = bp.second.exit_terminal;
    int end_x = target_exit.grid_coord_x;
    int end_y = target_exit.grid_coord_y;
    int end_layer = target_exit.layer;
    
    std::priority_queue<AStarNodeReroute, std::vector<AStarNodeReroute>, std::greater<AStarNodeReroute>> open_list;
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, int> best_g;
    std::unordered_set<AStarNodeReroute*> all_allocated_nodes;
    AStarResultReroute routing_result;

    AStarNodeReroute start_node(start_exit.grid_coord_x, start_exit.grid_coord_y, start_exit.layer);
    start_node.h = grid_distance_reroute(
        start_exit.grid_coord_x, start_exit.grid_coord_y, start_exit.layer,
        end_x, end_y, end_layer
    );
    start_node.f = start_node.g + start_node.h;
    open_list.push(start_node);
    best_g[get_coord_node_key_reroute(start_node.x, start_node.y, start_node.layer)] = 0;

    while (!open_list.empty()) {
        AStarNodeReroute current = open_list.top();
        // // std::cout << "current g: " << current.g << "\n";
        open_list.pop();
        std::string current_key = get_coord_node_key_reroute(current.x, current.y, current.layer);

        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            // // std::cout << "Path found with length: " << current.g << "\n";
            routing_result.path = reconstruct_path_reroute(&current, occupied_set);
            routing_result.nodes_to_delete = std::move(all_allocated_nodes);
            return routing_result;
        }

        if (closed_set.count(current_key) && best_g[current_key] < current.g) {
            continue;
        }
        closed_set.insert(current_key);

        std::vector<AStarNodeReroute> neighbors = get_neighbors_reroute(current, max_width, max_height, layer_count, occupied_set, end_x, end_y, end_layer);
        for (AStarNodeReroute& neighbor : neighbors) {
            std::string neighbor_key = get_coord_node_key_reroute(neighbor.x, neighbor.y, neighbor.layer);
            if (closed_set.count(neighbor_key)) {
                continue;
            }

            int tentative_g = current.g + 1;
            if (!best_g.count(neighbor_key) || tentative_g < best_g[neighbor_key]) {
                neighbor.parent = new AStarNodeReroute(current);
                all_allocated_nodes.insert(neighbor.parent);
                neighbor.g = tentative_g;
                neighbor.h = grid_distance_reroute(
                    neighbor.x, neighbor.y, neighbor.layer,
                    end_x, end_y, end_layer
                );
                neighbor.f = neighbor.g + neighbor.h;

                open_list.push(neighbor);
                best_g[neighbor_key] = tentative_g;
            }
        }
    }

    routing_result.path = {};
    routing_result.nodes_to_delete = std::move(all_allocated_nodes);
    return routing_result;
}

json coord_to_json_reroute(int x, int y, int layer) {
    std::string layer_name = (layer == 0) ? "Bottom" : "Top";
    return json::array({x, y, layer_name});
}


bool is_same_direction_reroute(int dx1, int dy1, int dl1, int dx2, int dy2, int dl2) {
    return (dx1 == dx2) && (dy1 == dy2) && (dl1 == dl2);
}


std::vector<std::pair<ResultPoint, ResultPoint>> split_to_segments_reroute(const std::vector<AStarNodeReroute>& path) {
    std::vector<std::pair<ResultPoint, ResultPoint>> segments;
    if (path.size() < 2) {
        return segments;
    }

    ResultPoint current_start = {path[0].x, path[0].y, path[0].layer};
    int prev_dx = path[1].x - path[0].x;
    int prev_dy = path[1].y - path[0].y;
    int prev_dl = path[1].layer - path[0].layer;

    for (size_t i = 2; i < path.size(); ++i) {
        const AStarNodeReroute& prev_node = path[i-1];
        const AStarNodeReroute& curr_node = path[i];

        int curr_dx = curr_node.x - prev_node.x;
        int curr_dy = curr_node.y - prev_node.y;
        int curr_dl = curr_node.layer - prev_node.layer;

        if (!is_same_direction_reroute(prev_dx, prev_dy, prev_dl, curr_dx, curr_dy, curr_dl)) {
            ResultPoint new_node = {prev_node.x, prev_node.y, prev_node.layer};
            segments.emplace_back(current_start, new_node);
            current_start = new_node;
            prev_dx = curr_dx;
            prev_dy = curr_dy;
            prev_dl = curr_dl;
        }
    }

    const AStarNodeReroute& last_node = path.back();
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

std::string get_node_key_2(float x, float y, int layer) {
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(layer);
}

ExitBumpPos getExitBumpBorder(const PairTeam &pt) {
    ExitBumpPos ebp;
    for (const BumpPair &bp : pt.member) {
        // // std::cout << "Assigning exit border for BumpPair " << bp.first.net_name << ": ";
        // // std::cout << "=> from (" << bp.first.grid_coord_x << ", " << bp.first.grid_coord_y << ") to ("
        //           << bp.first.exit_terminal.grid_coord_x << ", " << bp.first.exit_terminal.grid_coord_y << ")" << std::endl;
        
        if (bp.first.exit_terminal.grid_coord_x == pt.start_left_bound_x) ebp.first.push_back(LEFT);
        else if (bp.first.exit_terminal.grid_coord_x == pt.start_right_bound_x) ebp.first.push_back(RIGHT);
        else if (bp.first.exit_terminal.grid_coord_y == pt.start_up_bound_y) ebp.first.push_back(DOWN);
        else if (bp.first.exit_terminal.grid_coord_y == pt.start_down_bound_y) ebp.first.push_back(UP);
        else IOUtils::terminateProgram("ERROR: Wrong Border Assignment!");
        
        // // std::cout << "=> from (" << bp.second.grid_coord_x << ", " << bp.second.grid_coord_y << ") to ("
        //           << bp.second.exit_terminal.grid_coord_x << ", " << bp.second.exit_terminal.grid_coord_y << ")" << std::endl;

        if (bp.second.exit_terminal.grid_coord_x == pt.end_left_bound_x) ebp.second.push_back(LEFT);
        else if (bp.second.exit_terminal.grid_coord_x == pt.end_right_bound_x) ebp.second.push_back(RIGHT);
        else if (bp.second.exit_terminal.grid_coord_y == pt.end_up_bound_y) ebp.second.push_back(DOWN);
        else if (bp.second.exit_terminal.grid_coord_y == pt.end_down_bound_y) ebp.second.push_back(UP);
        else IOUtils::terminateProgram("ERROR: Wrong Border Assignment!");
    }
    return ebp;
}

bool canDirectRoute(const ResultPoint &a, const ResultPoint &b) {
    if (a.x == b.x || a.y == b.y || std::abs(a.y-b.y) == std::abs(a.x-b.x)) return true;
    return false;
}

bool canBendOneTime(ExitTerminal start, ExitPos start_pos, ExitTerminal end, ExitPos end_pos, bool &outStart, ResultPoint &cp) {
    if (!outStart) {
        std::swap(start, end);
        std::swap(start_pos, end_pos);
    }

    if (start_pos == end_pos) return false;
    if (start_pos == RIGHT && end_pos == LEFT && start.grid_coord_x > end.grid_coord_x) return false;
    if (start_pos == LEFT && end_pos == RIGHT && start.grid_coord_x < end.grid_coord_x) return false;
    if (start_pos == UP && end_pos == DOWN && start.grid_coord_y > end.grid_coord_y) return false;
    if (start_pos == DOWN && end_pos == UP && start.grid_coord_y < end.grid_coord_y) return false;

    int run_time = 0;
    while (++run_time < 3) {
        if (start_pos == UP) {
            cp.x = start.grid_coord_x;
            cp.y = end.grid_coord_y - std::abs(start.grid_coord_x - end.grid_coord_x);
            if (cp.y > start.grid_coord_y) return true;
        } else if (start_pos == RIGHT) {
            cp.y = start.grid_coord_y;
            cp.x = end.grid_coord_x - std::abs(start.grid_coord_y - end.grid_coord_y);
            if (cp.x > start.grid_coord_x) return true;
        } else if (start_pos == LEFT) {
            cp.y = start.grid_coord_y;
            cp.x = end.grid_coord_x + std::abs(start.grid_coord_y - end.grid_coord_y);
            if (cp.x < start.grid_coord_x) return true;
        } else {
            cp.x = start.grid_coord_x;
            cp.y = end.grid_coord_y + std::abs(start.grid_coord_x - end.grid_coord_x);
            if (cp.y < start.grid_coord_y) return true;
        }
        std::swap(start, end);
        std::swap(start_pos, end_pos);
        outStart = !outStart;
    }

    return false;
}

bool canBendTwoTime(const ExitTerminal &start, ExitPos start_pos, const ExitTerminal &end, ExitPos end_pos, ResultPoint &cp1, ResultPoint &cp2, int bend_time) {
    int abs_y_diff = std::abs(start.grid_coord_y - end.grid_coord_y);
    int abs_x_diff = std::abs(start.grid_coord_x - end.grid_coord_x);

    if (start_pos == LEFT && end_pos == RIGHT && abs_x_diff < abs_y_diff) {
        cp2.x = cp1.x = start.grid_coord_x - 2 * bend_time;
        if (cp1.x <= end.grid_coord_x) IOUtils::terminateProgram("No left space to place more lines!");
        // p = 1: start higher
        int p = std::abs(start.grid_coord_y - end.grid_coord_y) / (start.grid_coord_y - end.grid_coord_y);
        cp1.y = start.grid_coord_y - p * 2 * bend_time;
        cp2.y = end.grid_coord_y + p * std::abs(end.grid_coord_x - cp1.x);
        return true;
    } else if (start_pos == RIGHT && end_pos == LEFT && abs_x_diff < abs_y_diff) {
        cp2.x = cp1.x = start.grid_coord_x + 2 * bend_time;
        if (cp1.x >= end.grid_coord_x) IOUtils::terminateProgram("No left space to place more lines!");
        // p = 1: start higher
        int p = std::abs(start.grid_coord_y - end.grid_coord_y) / (start.grid_coord_y - end.grid_coord_y);
        cp1.y = start.grid_coord_y - p * 2 * bend_time;
        cp2.y = end.grid_coord_y + p * std::abs(end.grid_coord_x - cp1.x);
        return true;
    } else if (start_pos == UP && end_pos == DOWN && abs_x_diff > abs_y_diff) {
        // p = 1: start righter
        int p = std::abs(start.grid_coord_x - end.grid_coord_x) / (start.grid_coord_x - end.grid_coord_x);
        cp1.y = cp2.y = start.grid_coord_y + 2 * bend_time;
        if (cp1.y >= end.grid_coord_y) IOUtils::terminateProgram("No left space to place more lines!");
        cp1.x = start.grid_coord_x - p * 2 * bend_time;
        cp2.x = end.grid_coord_x + p * std::abs(end.grid_coord_y - cp1.y);
        return true;
    } else if (start_pos == DOWN && end_pos == UP && abs_x_diff > abs_y_diff) {
        // p = 1: start righter
        int p = std::abs(start.grid_coord_x - end.grid_coord_x) / (start.grid_coord_x - end.grid_coord_x);
        cp1.y = cp2.y = start.grid_coord_y - 2 * bend_time;
        if (cp1.y <= end.grid_coord_y) IOUtils::terminateProgram("No left space to place more lines!");
        cp1.x = start.grid_coord_x - p * 2 * bend_time;
        cp2.x = end.grid_coord_x + p * std::abs(end.grid_coord_y - cp1.y);
        return true;
    }

    return false;
}

bool update_occupied_set(std::vector<Seg> &PerNetSeg, RoutingCase& rc) {
    std::vector<std::string> inserting_keys;
    int seg_count = PerNetSeg.size();
    for (int seg_idx = 0; seg_idx < seg_count; ++seg_idx) {
        Seg &seg = PerNetSeg[seg_idx];
        ResultPoint &start = seg.first;
        ResultPoint &end = seg.second;
        int dx = end.x - start.x;
        int dy = end.y - start.y;
        int step_x = dx == 0 ? 0 : (dx > 0 ? 1 : -1);
        int step_y = dy == 0 ? 0 : (dy > 0 ? 1 : -1);
        int total_steps = std::max(abs(dx), abs(dy));
        int curr_x = start.x;
        int curr_y = start.y;
        if (dx != 0 && dy != 0) {
            for (int i = 0; i < total_steps; ++i) {
                std::string node_key = get_node_key_2(static_cast<float>(curr_x), static_cast<float>(curr_y), start.layer);
                if (rc.occupied_set.find(node_key) == rc.occupied_set.end()) {
                    inserting_keys.push_back(node_key);
                } else {
                    if (!(
                        (seg_idx == 0 && node_key == get_node_key_2(static_cast<float>(start.x), static_cast<float>(start.y), start.layer)) ||
                        (seg_idx == seg_count - 1 && node_key == get_node_key_2(static_cast<float>(end.x), static_cast<float>(end.y), end.layer))
                    )) {
                       return false;
                    }
                }
                float mid_x = static_cast<float>(curr_x) + static_cast<float>(step_x) * 0.5f;
                float mid_y = static_cast<float>(curr_y) + static_cast<float>(step_y) * 0.5f;
                std::string mid_key = get_node_key_2(mid_x, mid_y, start.layer);
                if (rc.occupied_set.find(mid_key) == rc.occupied_set.end()) {
                    inserting_keys.push_back(mid_key);
                } else {
                    if (!(
                        (seg_idx == 0 && mid_key == get_node_key_2(static_cast<float>(start.x), static_cast<float>(start.y), start.layer)) ||
                        (seg_idx == seg_count - 1 && mid_key == get_node_key_2(static_cast<float>(end.x), static_cast<float>(end.y), end.layer))
                    )) {
                       return false;
                    }
                }
                curr_x += step_x;
                curr_y += step_y;
            }
            std::string last_node_key = get_node_key_2(static_cast<float>(end.x), static_cast<float>(end.y), start.layer);
            if (rc.occupied_set.find(last_node_key) == rc.occupied_set.end()) {
                inserting_keys.push_back(last_node_key);
            } else {
                if (!(
                    (seg_idx == 0 && last_node_key == get_node_key_2(static_cast<float>(start.x), static_cast<float>(start.y), start.layer)) ||
                    (seg_idx == seg_count - 1 && last_node_key == get_node_key_2(static_cast<float>(end.x), static_cast<float>(end.y), end.layer))
                )) {
                    return false;
                }
            }
        } else {
            for (int i = 0; i <= total_steps; ++i) {
                std::string node_key = get_node_key_2(static_cast<float>(curr_x), static_cast<float>(curr_y), start.layer);
                if (rc.occupied_set.find(node_key) == rc.occupied_set.end()) {
                    inserting_keys.push_back(node_key);
                } else {
                    if (!( 
                        (seg_idx == 0 && node_key == get_node_key_2(static_cast<float>(start.x), static_cast<float>(start.y), start.layer)) ||
                        (seg_idx == seg_count - 1 && node_key == get_node_key_2(static_cast<float>(end.x), static_cast<float>(end.y), end.layer))
                    )) {
                       return false;
                    }
                }
                curr_x += step_x;
                curr_y += step_y;
            }
        }
    }
    for (const std::string& key : inserting_keys) {
        rc.occupied_set.insert(key);
    }
    return true;
}

DirectRoutingResult direct_route(const std::vector<PairTeam>& vpt, RoutingCase& rc) {
    DirectRoutingResult res;
    DRResultDict ns;
    std::vector<BumpPair> failed_nets;
    std::vector<std::string> final_failed_nets;
    for (const PairTeam &pt : vpt) {
        if (pt.member[0].first.exit_terminal == pt.member[0].second.exit_terminal) {
            for (const BumpPair & bp : pt.member) {
                std::vector<Seg> PerNetSeg;
                ns[bp.first.net_name] = PerNetSeg;
            }
            continue;
        }

        ExitBumpPos ebp = getExitBumpBorder(pt);
        bool routeStart = true;
        int twobendtimes = 0;
        for (size_t i = 0; i < pt.member.size(); i++) {
            std::vector<Seg> PerNetSeg;
            const BumpPair bp = pt.member[i];
            const std::string& net_name = bp.first.net_name;
            ResultPoint rp1{bp.first.exit_terminal.grid_coord_x, bp.first.exit_terminal.grid_coord_y, bp.first.exit_terminal.layer};
            ResultPoint rp2{bp.second.exit_terminal.grid_coord_x, bp.second.exit_terminal.grid_coord_y, bp.second.exit_terminal.layer};
            if (canDirectRoute(rp1, rp2)) {
                PerNetSeg.emplace_back(rp1, rp2);
                ns[net_name] = PerNetSeg;
                twobendtimes = 0;
            } else {
                ResultPoint changePoint1{-1, -1, bp.first.exit_terminal.layer};
                ResultPoint changePoint2{-1, -1, bp.first.exit_terminal.layer};
                if (canBendOneTime(bp.first.exit_terminal, ebp.first[i], bp.second.exit_terminal, ebp.second[i], routeStart, changePoint1)) {
                    PerNetSeg.emplace_back(rp1, changePoint1);
                    PerNetSeg.emplace_back(changePoint1, rp2);
                    ns[net_name] = PerNetSeg;
                    twobendtimes = 0;
                } else if (canBendTwoTime(bp.first.exit_terminal, ebp.first[i], bp.second.exit_terminal, ebp.second[i], changePoint1, changePoint2, ++twobendtimes)) {
                    PerNetSeg.emplace_back(rp1, changePoint1);
                    PerNetSeg.emplace_back(changePoint1, changePoint2);
                    PerNetSeg.emplace_back(changePoint2, rp2);
                    ns[net_name] = PerNetSeg;
                } else {
                    IOUtils::terminateProgram("ERROR: Cannot only bend twice when direct routing.");
                }
            }
            bool routing_success = update_occupied_set(PerNetSeg, rc);
            if (!routing_success) {
                failed_nets.push_back(bp);
            }
        }
    }
    for (const BumpPair& bp : failed_nets) {
        auto reroute_result = a_star_find_path_reroute(bp, rc.grid_max_width, rc.grid_max_height, rc.layer_count, rc.occupied_set);
        if (reroute_result.path.empty() && !(bp.first.exit_terminal.grid_coord_x == bp.second.exit_terminal.grid_coord_x && bp.first.exit_terminal.grid_coord_y == bp.second.exit_terminal.grid_coord_y)) {
            final_failed_nets.push_back(bp.first.net_name);
            ns[bp.first.net_name] = {};
        } else {
            std::vector<Seg> PerNetSeg = split_to_segments_reroute(reroute_result.path);
            ns[bp.first.net_name] = PerNetSeg;
        }
        for (AStarNodeReroute* node_ptr : reroute_result.nodes_to_delete) {
            delete node_ptr;
        }
        reroute_result.nodes_to_delete.clear();
    }
    res.dr_result_dict = ns;
    res.final_failed_nets = final_failed_nets;
    return res;
}