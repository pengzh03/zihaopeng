#include "a_star_direct.h"
#include "builder.h"
#include "elements.h"
#include "ThreadPool.h"
#include <mutex>

AStarNodeDirect::AStarNodeDirect(int x_, int y_, int layer_)
    : x(x_), y(y_), layer(layer_), g(0), h(0), f(0), parent(nullptr) {}


bool AStarNodeDirect::operator>(const AStarNodeDirect& other) const {
    return f > other.f;
}


std::string get_coord_node_key_direct(int x, int y, int layer) {
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(layer);
}


std::string get_node_key_direct(float x, float y, int layer) {
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(layer);
}


int grid_distance_direct(int x1, int y1, int l1, int x2, int y2, int l2) {
    return std::max(abs(x1 - x2), abs(y1 - y2)) + abs(l1 - l2);
}

//====================================Single Thread====================================

std::vector<AStarNodeDirect> get_neighbors_direct(const AStarNodeDirect& current, int max_width, int max_height, int layer_count, const std::unordered_set<std::string>& occupied_set, int end_x, int end_y, int end_layer) {
    std::vector<AStarNodeDirect> neighbors;
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
            std::string coord = get_node_key_direct(new_x, new_y, new_layer);
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
                std::string key = get_node_key_direct(mid_x, mid_y, new_layer);
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


std::vector<AStarNodeDirect> reconstruct_path_direct(AStarNodeDirect* end_node, std::unordered_set<std::string>& occupied_set) {
    std::vector<AStarNodeDirect> path;
    AStarNodeDirect* current = end_node;
    AStarNodeDirect* last = end_node;
    while (current != nullptr) {
        path.push_back(*current);
        std::string last_node_key = get_node_key_direct(current->x, current->y, current->layer);
        if (occupied_set.find(last_node_key) == occupied_set.end()) {
            occupied_set.insert(last_node_key);
        }
        if ((last->x != current->x) && (last->y != current->y)) {
            float mid_x = static_cast<float>(last->x + current->x) * 0.5f;
            float mid_y = static_cast<float>(last->y + current->y) * 0.5f;
            std::string mid_node_key = get_node_key_direct(mid_x, mid_y, current->layer);
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


AStarResultDirect a_star_find_path_direct(const Bump& start, int max_width, int max_height, int layer_count, std::unordered_set<std::string>& occupied_set) {
    const ExitTerminal& target_exit = start.exit_terminal;
    int end_x = target_exit.grid_coord_x;
    int end_y = target_exit.grid_coord_y;
    int end_layer = target_exit.layer;
    
    std::priority_queue<AStarNodeDirect, std::vector<AStarNodeDirect>, std::greater<AStarNodeDirect>> open_list;
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, int> best_g;
    std::unordered_set<AStarNodeDirect*> all_allocated_nodes;
    AStarResultDirect routing_result;

    AStarNodeDirect start_node(start.grid_coord_x, start.grid_coord_y, start.layer);
    start_node.h = grid_distance_direct(
        start.grid_coord_x, start.grid_coord_y, start.layer,
        end_x, end_y, end_layer
    );
    start_node.f = start_node.g + start_node.h;
    open_list.push(start_node);
    best_g[get_coord_node_key_direct(start_node.x, start_node.y, start_node.layer)] = 0;

    while (!open_list.empty()) {
        AStarNodeDirect current = open_list.top();
        // std::cout << "current g: " << current.g << "\n";
        open_list.pop();
        std::string current_key = get_coord_node_key_direct(current.x, current.y, current.layer);

        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            // std::cout << "Path found with length: " << current.g << "\n";
            routing_result.path = reconstruct_path_direct(&current, occupied_set);
            routing_result.nodes_to_delete = std::move(all_allocated_nodes);
            return routing_result;
        }

        if (closed_set.count(current_key) && best_g[current_key] < current.g) {
            continue;
        }
        closed_set.insert(current_key);

        std::vector<AStarNodeDirect> neighbors = get_neighbors_direct(current, max_width, max_height, layer_count, occupied_set, end_x, end_y, end_layer);
        for (AStarNodeDirect& neighbor : neighbors) {
            std::string neighbor_key = get_coord_node_key_direct(neighbor.x, neighbor.y, neighbor.layer);
            if (closed_set.count(neighbor_key)) {
                continue;
            }

            int tentative_g = current.g + 1;
            if (!best_g.count(neighbor_key) || tentative_g < best_g[neighbor_key]) {
                neighbor.parent = new AStarNodeDirect(current);
                all_allocated_nodes.insert(neighbor.parent);
                neighbor.g = tentative_g;
                neighbor.h = grid_distance_direct(
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

json coord_to_json_direct(int x, int y, int layer) {
    std::string layer_name = (layer == 0) ? "Bottom" : "Top";
    return json::array({x, y, layer_name});
}


bool is_same_direction_direct(int dx1, int dy1, int dl1, int dx2, int dy2, int dl2) {
    return (dx1 == dx2) && (dy1 == dy2) && (dl1 == dl2);
}


std::vector<std::pair<ResultPoint, ResultPoint>> split_to_segments_direct(const std::vector<AStarNodeDirect>& path) {
    std::vector<std::pair<ResultPoint, ResultPoint>> segments;
    if (path.size() < 2) {
        return segments;
    }

    ResultPoint current_start = {path[0].x, path[0].y, path[0].layer};
    int prev_dx = path[1].x - path[0].x;
    int prev_dy = path[1].y - path[0].y;
    int prev_dl = path[1].layer - path[0].layer;

    for (size_t i = 2; i < path.size(); ++i) {
        const AStarNodeDirect& prev_node = path[i-1];
        const AStarNodeDirect& curr_node = path[i];

        int curr_dx = curr_node.x - prev_node.x;
        int curr_dy = curr_node.y - prev_node.y;
        int curr_dl = curr_node.layer - prev_node.layer;

        if (!is_same_direction_direct(prev_dx, prev_dy, prev_dl, curr_dx, curr_dy, curr_dl)) {
            ResultPoint new_node = {prev_node.x, prev_node.y, prev_node.layer};
            segments.emplace_back(current_start, new_node);
            current_start = new_node;
            prev_dx = curr_dx;
            prev_dy = curr_dy;
            prev_dl = curr_dl;
        }
    }

    const AStarNodeDirect& last_node = path.back();
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

void printFailedNets(std::vector<std::basic_string<char>> &reroute_fail_nets) {
    if (reroute_fail_nets.empty()) return;
    std::cout << "Reroute failed for net: ";
    for (const auto& net_name : reroute_fail_nets) {
        std::cout << net_name << ", ";
    }
    std::cout << "\n";
}

RerouteResult a_star_reroute(
    RoutingCase& rc,
    const std::vector<PairTeam>& vpt,
    std::unordered_map<int, std::vector<std::pair<size_t, int>>> all_fail_bumps
) {
    std::vector<std::string> reroute_fail_nets;
    std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> escape_results;
    for (size_t team_idx = 0; team_idx < vpt.size(); ++team_idx) {
        if (all_fail_bumps.find(team_idx) == all_fail_bumps.end()) {
            continue;
        }
        const std::vector<std::pair<size_t, int>>& fail_bumps = all_fail_bumps[team_idx];
        const PairTeam& pt = vpt[team_idx];
        for (const auto& fail_pair : fail_bumps) {
            size_t pair_idx = fail_pair.first;
            int bump_idx = fail_pair.second;
            const BumpPair& bump_pair = pt.member[pair_idx];
            const std::string& net_name = bump_pair.first.net_name;
            const std::string& fail_bump_name = rc.net_dict.at(net_name).bump_names[bump_idx];
            const Bump& fail_bump = rc.bump_dict.at(fail_bump_name);
            AStarResultDirect routing_result = a_star_find_path_direct(fail_bump, rc.grid_max_width, rc.grid_max_height, rc.layer_count, rc.occupied_set);
            if (routing_result.path.empty() && !(fail_bump.grid_coord_x == fail_bump.exit_terminal.grid_coord_x && fail_bump.grid_coord_y == fail_bump.exit_terminal.grid_coord_y)) {
                reroute_fail_nets.push_back(net_name);
            }
            std::vector<std::pair<ResultPoint, ResultPoint>> new_segments = split_to_segments_direct(routing_result.path);
            for (AStarNodeDirect* node : routing_result.nodes_to_delete) {
                delete node;
            }
            routing_result.nodes_to_delete.clear();
            escape_results[fail_bump_name].insert(
                escape_results[fail_bump_name].end(),
                new_segments.begin(),
                new_segments.end()
            );
        }
    }
    RerouteResult result;
    result.reroute_fail_nets = reroute_fail_nets;
    result.escape_results = escape_results;
    // printFailedNets(reroute_fail_nets);
    return result;
}

//====================================Multi Thread====================================
std::vector<AStarNodeDirect> get_neighbors_direct_lock(
        const AStarNodeDirect& current,
        int max_width, int max_height, int layer_count,
        const std::unordered_set<std::string>& occupied_set,
        int end_x, int end_y, int end_layer,
        std::mutex& rc_mutex  // 新增：保护occupied_set的锁
) {
    std::vector<AStarNodeDirect> neighbors;
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
            std::string coord = get_node_key_direct(new_x, new_y, new_layer);
            // 读操作加锁
            bool coord_occupied = false;
            {
                std::lock_guard<std::mutex> lock(rc_mutex);
                coord_occupied = (occupied_set.find(coord) != occupied_set.end());
            }
            if (coord_occupied && !(new_x == end_x && new_y == end_y && new_layer == end_layer)) {
                continue;
            }

            if (dx != 0 && dy != 0) {
                if (current.parent != nullptr && ((new_x == current.parent->x && current.y == current.parent->y) || (new_y == current.parent->y && current.x == current.parent->x))) {
                    continue;
                }
                float mid_x = static_cast<float>(current.x) + static_cast<float>(dx) * 0.5f;
                float mid_y = static_cast<float>(current.y) + static_cast<float>(dy) * 0.5f;
                std::string key = get_node_key_direct(mid_x, mid_y, new_layer);
                // 读操作加锁
                bool key_occupied = false;
                {
                    std::lock_guard<std::mutex> lock(rc_mutex);
                    key_occupied = (occupied_set.find(key) != occupied_set.end());
                }
                if (key_occupied) {
                    continue;
                }
            } else {
                if (dx != 0) {
                    if (current.parent != nullptr && new_x == current.parent->x) {
                        continue;
                    }
                } else if (dy != 0) {
                    if (current.parent != nullptr && new_y == current.parent->y) {
                        continue;
                    }
                }
            }
            neighbors.emplace_back(new_x, new_y, new_layer);
        }
    }
    return neighbors;
}

std::vector<AStarNodeDirect> reconstruct_path_direct_lock(
        AStarNodeDirect* end_node,
        std::unordered_set<std::string>& occupied_set,
        std::mutex& rc_mutex  // 新增：保护occupied_set的锁
) {
    std::vector<AStarNodeDirect> path;
    AStarNodeDirect* current = end_node;
    AStarNodeDirect* last = end_node;
    while (current != nullptr) {
        path.push_back(*current);
        // 写操作加锁
        {
            std::lock_guard<std::mutex> lock(rc_mutex);
            occupied_set.insert(get_node_key_direct(current->x, current->y, current->layer));
        }
        if ((last->x != current->x) && (last->y != current->y)) {
            float mid_x = static_cast<float>(last->x + current->x) * 0.5f;
            float mid_y = static_cast<float>(last->y + current->y) * 0.5f;
            // 写操作加锁
            {
                std::lock_guard<std::mutex> lock(rc_mutex);
                occupied_set.insert(get_node_key_direct(mid_x, mid_y, current->layer));
            }
        }
        last = current;
        current = current->parent;
    }
    return path;
}

AStarResultDirect a_star_find_path_direct_lock(
        const Bump& start,
        int max_width, int max_height, int layer_count,
        std::unordered_set<std::string>& occupied_set,
        std::mutex& rc_mutex  // 新增：传递给子函数的锁
) {
    const ExitTerminal& target_exit = start.exit_terminal;
    int end_x = target_exit.grid_coord_x;
    int end_y = target_exit.grid_coord_y;
    int end_layer = target_exit.layer;

    std::priority_queue<AStarNodeDirect, std::vector<AStarNodeDirect>, std::greater<AStarNodeDirect>> open_list;
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, int> best_g;
    std::unordered_set<AStarNodeDirect*> all_allocated_nodes;
    AStarResultDirect routing_result;

    AStarNodeDirect start_node(start.grid_coord_x, start.grid_coord_y, start.layer);
    start_node.h = grid_distance_direct(
            start.grid_coord_x, start.grid_coord_y, start.layer,
            end_x, end_y, end_layer
    );
    start_node.f = start_node.g + start_node.h;
    open_list.push(start_node);
    best_g[get_coord_node_key_direct(start_node.x, start_node.y, start_node.layer)] = 0;

    while (!open_list.empty()) {
        AStarNodeDirect current = open_list.top();
        open_list.pop();
        std::string current_key = get_coord_node_key_direct(current.x, current.y, current.layer);

        if (current.x == end_x && current.y == end_y && current.layer == end_layer) {
            // 调用加锁后的reconstruct_path
            routing_result.path = reconstruct_path_direct_lock(&current, occupied_set, rc_mutex);
            routing_result.nodes_to_delete = std::move(all_allocated_nodes);
            return routing_result;
        }

        if (closed_set.count(current_key) && best_g[current_key] < current.g) {
            continue;
        }
        closed_set.insert(current_key);

        // 调用加锁后的get_neighbors
        std::vector<AStarNodeDirect> neighbors = get_neighbors_direct_lock(
                current, max_width, max_height, layer_count, occupied_set, end_x, end_y, end_layer, rc_mutex
        );
        for (AStarNodeDirect& neighbor : neighbors) {
            std::string neighbor_key = get_coord_node_key_direct(neighbor.x, neighbor.y, neighbor.layer);
            if (closed_set.count(neighbor_key)) {
                continue;
            }

            int tentative_g = current.g + 1;
            if (!best_g.count(neighbor_key) || tentative_g < best_g[neighbor_key]) {
                neighbor.parent = new AStarNodeDirect(current);
                all_allocated_nodes.insert(neighbor.parent);
                neighbor.g = tentative_g;
                neighbor.h = grid_distance_direct(
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


void a_star_reroute_per_team(
        RoutingCase& rc,
        const PairTeam& pt,
        const std::vector<std::pair<size_t, int>>& fail_bumps,
        std::vector<std::string> &reroute_fail_nets,
        std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> &escape_results,
        std::mutex& fail_nets_mutex,    // 新增：保护reroute_fail_nets的锁
        std::mutex& escape_results_mutex,  // 新增：保护escape_results的锁
        std::mutex& rc_mutex
)
{
    for (const auto& fail_pair : fail_bumps) {
        size_t pair_idx = fail_pair.first;
        int bump_idx = fail_pair.second;
        const BumpPair& bump_pair = pt.member[pair_idx];
        const std::string& net_name = bump_pair.first.net_name;
        const std::string& fail_bump_name = rc.net_dict.at(net_name).bump_names[bump_idx];
        const Bump& fail_bump = rc.bump_dict.at(fail_bump_name);

        AStarResultDirect routing_result = a_star_find_path_direct_lock(
                fail_bump,
                rc.grid_max_width,
                rc.grid_max_height,
                rc.layer_count,
                rc.occupied_set,
                rc_mutex
        );

        // 1. 保护reroute_fail_nets的push_back操作
        if (routing_result.path.empty() && !(fail_bump.grid_coord_x == fail_bump.exit_terminal.grid_coord_x && fail_bump.grid_coord_y == fail_bump.exit_terminal.grid_coord_y)) {
            std::lock_guard<std::mutex> lock(fail_nets_mutex); // 自动加锁，作用域结束解锁
            reroute_fail_nets.push_back(net_name);
        }

        std::vector<std::pair<ResultPoint, ResultPoint>> new_segments = split_to_segments_direct(routing_result.path);

        // 释放内存（无共享资源，无需加锁）
        for (AStarNodeDirect* node : routing_result.nodes_to_delete) {
            delete node;
        }
        routing_result.nodes_to_delete.clear();

        // 2. 保护escape_results的insert操作
        std::lock_guard<std::mutex> escape_lock(escape_results_mutex);
        escape_results[fail_bump_name].insert(
                escape_results[fail_bump_name].end(),
                new_segments.begin(),
                new_segments.end()
        );
    }
}

RerouteResult a_star_reroute_multithread(
        RoutingCase& rc,
        const std::vector<PairTeam>& vpt,
        std::unordered_map<int, std::vector<std::pair<size_t, int>>> all_fail_bumps
) {
    std::vector<std::string> reroute_fail_nets;
    std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> escape_results;

    // 初始化互斥锁：分别保护不同共享资源
    std::mutex fail_nets_mutex;
    std::mutex escape_results_mutex;
    std::mutex rc_mutex;

    ThreadPool pool(4);
    std::vector<std::future<void>> futures;

    for (size_t team_idx = 0; team_idx < vpt.size(); ++team_idx) {
        auto fail_bumps_iter = all_fail_bumps.find(team_idx);
        if (fail_bumps_iter == all_fail_bumps.end()) {
            continue;
        }
        const std::vector<std::pair<size_t, int>>& fail_bumps = fail_bumps_iter->second;
        const PairTeam& pt = vpt[team_idx];

        // 线程函数：值捕获不可变数据，引用捕获锁和共享资源
        futures.emplace_back(
                pool.enqueue([&, pt, fail_bumps, team_idx]() {
                    // IOUtils::print("Rerouting Team "+std::to_string(team_idx)+"...\n");
                    a_star_reroute_per_team(
                            rc,
                            pt,
                            fail_bumps,
                            reroute_fail_nets,
                            escape_results,
                            fail_nets_mutex,       // 传入锁
                            escape_results_mutex,   // 传入锁
                            rc_mutex
                    );
                })
        );
    }

    // 等待所有线程执行完成
    for (auto& fut : futures) {
        fut.get();
    }

    RerouteResult result;
    result.reroute_fail_nets = reroute_fail_nets;
    result.escape_results = escape_results;
    // printFailedNets(reroute_fail_nets);
    return result;
}