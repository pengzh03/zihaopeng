#include "layer_assignment.h"
#include "ioutils.h"
#include <numeric>
#include <fstream>
#include <string>

// Calculate the occupancy area of a PairTeam
BusOccupancy calculate_occupancy(
    int start_left_bound_x,
    int start_right_bound_x,
    int start_up_bound_y,
    int start_down_bound_y,
    int end_left_bound_x,
    int end_right_bound_x,
    int end_up_bound_y,
    int end_down_bound_y
) {
    BusOccupancy occ;

    // // std::cout << "Calculating occupancy with bounds: "
    //           << "Start[" << start_left_bound_x << "," << start_right_bound_x << "]x["
    //           << start_up_bound_y << "," << start_down_bound_y << "] "
    //           << "End[" << end_left_bound_x << "," << end_right_bound_x << "]x["
    //           << end_up_bound_y << "," << end_down_bound_y << "]\n";

    occ.x_min = std::min(start_left_bound_x, end_left_bound_x);
    occ.x_max = std::max(start_right_bound_x, end_right_bound_x);
    occ.y_min = std::min(start_up_bound_y, end_up_bound_y);
    occ.y_max = std::max(start_down_bound_y, end_down_bound_y);

    std::vector<intPoint> start_rect_points = {
        {start_left_bound_x, start_up_bound_y},
        {start_right_bound_x, start_up_bound_y},
        {start_left_bound_x, start_down_bound_y},
        {start_right_bound_x, start_down_bound_y}
    };

    std::vector<intPoint> end_rect_points = {
        {end_left_bound_x, end_up_bound_y},
        {end_right_bound_x, end_up_bound_y},
        {end_left_bound_x, end_down_bound_y},
        {end_right_bound_x, end_down_bound_y}
    };

    std::unordered_map<int, int> start_corner_cuts;

    for (const intPoint& p : start_rect_points) {
        if (p.second != occ.y_min && p.second != occ.y_max) {
            if (p.first == occ.x_min) {
                if (p.second == start_up_bound_y) {
                    start_corner_cuts[1] = p.second - occ.y_min;
                } else { 
                    start_corner_cuts[2] = occ.y_max - p.second;
                }
            } else if (p.first == occ.x_max) {
                if (p.second == start_up_bound_y) {
                    start_corner_cuts[4] = p.second - occ.y_min;
                } else {
                    start_corner_cuts[3] = occ.y_max - p.second;
                }
            }
        } else if (p.first != occ.x_min && p.first != occ.x_max) {
            if (p.second == occ.y_min) {
                if (p.first == start_left_bound_x) {
                    start_corner_cuts[1] = p.first - occ.x_min;
                } else {
                    start_corner_cuts[4] = occ.x_max - p.first;
                }
            } else if (p.second == occ.y_max) {
                if (p.first == start_left_bound_x) {
                    start_corner_cuts[2] = p.first - occ.x_min;
                } else {
                    start_corner_cuts[3] = occ.x_max - p.first;
                }
            }
        }
    }

    for (const intPoint& p : end_rect_points) {
        if (p.second != occ.y_min && p.second != occ.y_max) {
            if (p.first == occ.x_min) {
                int update_position;
                int update_size;
                if (p.second == end_up_bound_y) {
                    update_position = 1;
                    update_size = p.second - occ.y_min;
                } else {
                    update_position = 2;
                    update_size = occ.y_max - p.second;
                }
                if (start_corner_cuts.find(update_position) != start_corner_cuts.end()) {
                    occ.corner_cuts[update_position] = std::min(start_corner_cuts[update_position], update_size);
                }
            } else if (p.first == occ.x_max) {
                int update_position;
                int update_size;
                if (p.second == end_up_bound_y) {
                    update_position = 4;
                    update_size = p.second - occ.y_min;
                } else {
                    update_position = 3;
                    update_size = occ.y_max - p.second;
                }
                if (start_corner_cuts.find(update_position) != start_corner_cuts.end()) {
                    occ.corner_cuts[update_position] = std::min(start_corner_cuts[update_position], update_size);
                }
            }
        } else if (p.first != occ.x_min && p.first != occ.x_max) {
            if (p.second == occ.y_min) {
                int update_position;
                int update_size;
                if (p.first == end_left_bound_x) {
                    update_position = 1;
                    update_size = p.first - occ.x_min;
                } else {
                    update_position = 4;
                    update_size = occ.x_max - p.first;
                }
                if (start_corner_cuts.find(update_position) != start_corner_cuts.end()) {
                    occ.corner_cuts[update_position] = std::min(start_corner_cuts[update_position], update_size);
                }
            } else if (p.second == occ.y_max) {
                int update_position;
                int update_size;
                if (p.first == end_left_bound_x) {
                    update_position = 2;
                    update_size = p.first - occ.x_min;
                } else {
                    update_position = 3;
                    update_size = occ.x_max - p.first;
                }
                if (start_corner_cuts.find(update_position) != start_corner_cuts.end()) {
                    occ.corner_cuts[update_position] = std::min(start_corner_cuts[update_position], update_size);
                }
            }
        }
    }

    return occ;
}

// Generate vertices of the convex polygon for the rectangle with cut corners (in clockwise order)
std::vector<Point> generate_cut_rect_vertices(const BusOccupancy& occ) {
    std::vector<Point> vertices;
    double x1 = static_cast<double>(occ.x_min);
    double x2 = static_cast<double>(occ.x_max);
    double y1 = static_cast<double>(occ.y_min);
    double y2 = static_cast<double>(occ.y_max);

    for (int corner_pos = 1;  corner_pos <= 4; corner_pos++) {
        if (occ.corner_cuts.count(corner_pos) == 0) {
            switch (corner_pos) {
                case 1: vertices.emplace_back(x1, y1); break;
                case 2: vertices.emplace_back(x1, y2); break;
                case 3: vertices.emplace_back(x2, y2); break;
                case 4: vertices.emplace_back(x2, y1); break;
            }
        } else {
            double s = static_cast<double>(occ.corner_cuts.at(corner_pos));
            switch (corner_pos) {
                case 1:
                    vertices.emplace_back(x1 + s, y1);
                    vertices.emplace_back(x1, y1 + s);
                    break;
                case 2:
                    vertices.emplace_back(x1, y2 - s);
                    vertices.emplace_back(x1 + s, y2);
                    break;
                case 3:
                    vertices.emplace_back(x2 - s, y2);
                    vertices.emplace_back(x2, y2 - s);
                    break;
                case 4:
                    vertices.emplace_back(x2, y1 + s);
                    vertices.emplace_back(x2 - s, y1);
                    break;
            }
        }
    }
    return vertices;
}

// Projection of a convex polygon on a specified axis
std::pair<double, double> LayerAssigner::project_convex_polygon(const std::vector<Point>& poly, const Point& axis) const {
    double min_proj = dot(poly[0], axis);
    double max_proj = min_proj;
    for (size_t i = 1; i < poly.size(); i++) {
        double proj = dot(poly[i], axis);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }
    return {min_proj, max_proj};
}

// Separating Axis Theorem for determining if two convex polygons intersect
bool LayerAssigner::convex_polygons_intersect(const std::vector<Point>& polyA, const std::vector<Point>& polyB) const {
    // Extract all edge normals of the two polygons (candidates for separating axes)
    auto getEdgeNormals = [&](const std::vector<Point>& poly) -> std::vector<Point> {
        std::vector<Point> normals;
        int n = poly.size();
        for (int i = 0; i < n; ++i) {
            Point edge = poly[(i+1)%n] - poly[i];
            Point normal = {-edge.second, edge.first}; // rotate 90 degrees
            normal = normalize(normal);
            normals.push_back(normal);
        }
        return normals;
    };

    std::vector<Point> normalsA = getEdgeNormals(polyA);
    std::vector<Point> normalsB = getEdgeNormals(polyB);

    // Check all separating axes: if there exists a non-overlapping projection, the polygons do not intersect
    auto checkAxis = [&](const Point& axis) -> bool {
        std::pair<double, double> projA = project_convex_polygon(polyA, axis);
        std::pair<double, double> projB = project_convex_polygon(polyB, axis);
        return !(projA.second < projB.first - 1e-6 || projB.second < projA.first - 1e-6);
    };

    for (const auto& axis : normalsA) {
        if (!checkAxis(axis)) return false;
    }
    for (const auto& axis : normalsB) {
        if (!checkAxis(axis)) return false;
    }

    return true;
}

// Bus conflict detection
bool LayerAssigner::bus_conflict(const std::vector<Point>& polyA, const std::vector<Point>& polyB) const {
    return convex_polygons_intersect(polyA, polyB);
}

// Build conflict graph based on bus conflicts
std::vector<std::vector<int>> LayerAssigner::build_conflict_graph(const std::vector<PairTeam>& vpt) const {
    int n = vpt.size();
    std::vector<std::vector<int>> conflict_graph(n);
    std::vector<std::vector<Point>> all_resources;

    int count = 0;
    for (const auto& pt : vpt) {
        count++;
        // std::cout << "Processing PairTeam " << count << "/" << n << std::endl;
        BusOccupancy occ = calculate_occupancy(
            pt.start_left_bound_x,
            pt.start_right_bound_x,
            pt.start_up_bound_y,
            pt.start_down_bound_y,
            pt.end_left_bound_x,
            pt.end_right_bound_x,
            pt.end_up_bound_y,
            pt.end_down_bound_y
        );
        // std::cout << "BusOccupancy: x[" << occ.x_min << "," << occ.x_max << "] y[" << occ.y_min << "," << occ.y_max << "] Cuts{";
        // for (const auto& cut : occ.corner_cuts) {
        //     // std::cout << "(" << cut.first << ":" << cut.second << ") ";
        // }
        // std::cout << "}" << std::endl;
        std::vector<Point> r_resource = generate_cut_rect_vertices(occ);
        all_resources.push_back(r_resource);
        // for (const auto& p : r_resource) {
        //     // std::cout << "(" << p.first << "," << p.second << ") ";
        // }
        // std::cout << std::endl;
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (bus_conflict(all_resources[i], all_resources[j])) {
                conflict_graph[i].push_back(j);
                conflict_graph[j].push_back(i);
                // // std::cout << "PairTeam " << (i + 1) << " and PairTeam " << (j + 1) << " Conflict" << std::endl;
            }
        }
    }

    return conflict_graph;
}

// Backtracking for optimal layer assignment
void LayerAssigner::backtrack(
    int current_idx,
    const std::vector<std::vector<int>>& conflict_graph,
    std::vector<int>& current_assignment,
    int current_max_layer,
    std::vector<int>& best_assignment,
    int& best_total_layers
) const {
    int n = conflict_graph.size();
    // all teams assigned, check if better than best
    if (current_idx == n) {
        int current_total = current_max_layer + 1;
        if (current_total < best_total_layers) {
            best_total_layers = current_total;
            best_assignment = current_assignment;
        }
        return;
    }
    // Pruning: current layer has reached or exceeded best layer, no need to continue
    if (current_max_layer + 1 >= best_total_layers) return;
    // Collect used layers from conflict nodes
    std::unordered_set<int> used_layers;
    for (int conflict_idx : conflict_graph[current_idx]) {
        if (current_assignment[conflict_idx] != -1) {
            used_layers.insert(current_assignment[conflict_idx]);
        }
    }
    // Try all possible layer numbers
    for (int layer = 0; layer < best_total_layers; ++layer) {
        if (used_layers.count(layer)) continue; // Skip used layers
        current_assignment[current_idx] = layer;
        backtrack(current_idx + 1, conflict_graph, current_assignment, 
                  std::max(current_max_layer, layer), best_assignment, best_total_layers);
        current_assignment[current_idx] = -1; // Backtrack: undo assignment
    }
}

// Get optimal layer assignment (backtracking)
LayerAssignmentResult LayerAssigner::assign_optimal_layers(const std::vector<PairTeam>& vpt) {
    LayerAssignmentResult result;
    int n = vpt.size();
    if (n == 0) {
        result.total_layers = 0;
        result.is_optimal = true;
        return result;
    }
    // IOUtils::print("Starting optimal layer assignment (backtracking)...\n");
    // IOUtils::print("Number of PairTeams: " + std::to_string(n) + "\n");
    // Build conflict graph
    auto conflict_graph = build_conflict_graph(vpt);
    // Initialize backtracking parameters
    std::vector<int> current_assignment(n, -1);  // -1 means unassigned
    std::vector<int> best_assignment(n, -1);
    int best_total_layers = INT_MAX;
    // Start backtracking
    backtrack(0, conflict_graph, current_assignment, -1, best_assignment, best_total_layers);
    result.team_layer = best_assignment;
    result.total_layers = best_total_layers;
    result.is_optimal = true;
    // IOUtils::print("Optimal layer assignment completed. Total layers needed: " + std::to_string(best_total_layers) + "\n");
    return result;
}

// Get approximate optimal solution (greedy)
LayerAssignmentResult LayerAssigner::assign_approximate_layers(const std::vector<PairTeam>& vpt) {
    LayerAssignmentResult result;
    int n = vpt.size();
    if (n == 0) {
        result.total_layers = 0;
        result.is_optimal = false;
        return result;
    }
    // IOUtils::print("Starting approximate layer assignment (greedy)...\n");
    // IOUtils::print("Number of PairTeams: " + std::to_string(n) + "\n");
    // Build conflict graph
    auto conflict_graph = build_conflict_graph(vpt);
    // Sort by conflict degree (prioritize teams with more conflicts)
    std::vector<int> team_indices(n);
    std::iota(team_indices.begin(), team_indices.end(), 0);
    std::sort(team_indices.begin(), team_indices.end(), [&](int a, int b) {
        return conflict_graph[a].size() > conflict_graph[b].size();
    });
    // Greedily assign layer numbers
    result.team_layer.resize(n, -1);
    int max_layer = 0;
    for (int idx : team_indices) {
        std::unordered_set<int> used_layers;
        for (int conflict_idx : conflict_graph[idx]) {
            if (result.team_layer[conflict_idx] != -1) {
                used_layers.insert(result.team_layer[conflict_idx]);
            }
        }
        // Assign smallest available layer number
        int assigned_layer = 0;
        while (used_layers.count(assigned_layer)) {
            assigned_layer++;
        }
        result.team_layer[idx] = assigned_layer;
        max_layer = std::max(max_layer, assigned_layer);
    }
    result.total_layers = max_layer + 1;
    result.is_optimal = false;
    // IOUtils::print("Approximate layer assignment completed. Total layers needed: " + std::to_string(result.total_layers) + "\n");
    return result;
}

LayerAssignmentResult LayerAssigner::assign_layers(std::vector<PairTeam>& vpt, RoutingCase& rc) {
    for (auto it = vpt.begin(); it != vpt.end(); ) {
        if (it->cnt == 0) {
            it = vpt.erase(it);
        } else {
            it->start_left_bound_x = std::max(0, it->start_left_bound_x - 5);
            it->start_right_bound_x = std::min(rc.grid_max_width - 1, it->start_right_bound_x + 5);
            it->start_up_bound_y = std::max(0, it->start_up_bound_y - 5);
            it->start_down_bound_y = std::min(rc.grid_max_height - 1, it->start_down_bound_y + 5);
            it->end_left_bound_x = std::max(0, it->end_left_bound_x - 5);
            it->end_right_bound_x = std::min(rc.grid_max_width - 1, it->end_right_bound_x + 5);
            it->end_up_bound_y = std::max(0, it->end_up_bound_y - 5);
            it->end_down_bound_y = std::min(rc.grid_max_height - 1, it->end_down_bound_y + 5);
            ++it;
        }
    }
    LayerAssignmentResult result;
    if (vpt.size() <= 20) {
        result = assign_optimal_layers(vpt);
    } else {
        result = assign_approximate_layers(vpt);
    }
    if (result.total_layers < 2) {
        result.total_layers = 2;
    }
    if (result.total_layers > 2) {
        rc.layer_count = result.total_layers;
        for (std::string net_name : rc.net_name_list) {
            Net& net = rc.net_dict.at(net_name);
            for (std::string bump_name : net.bump_names) {
                Bump& bump = rc.bump_dict.at(bump_name);
                if (bump.layer == 1) {
                    bump.layer = rc.layer_count - 1;
                }
            }
        }
        for (PairTeam& pt : vpt) {
            for (auto& bump_pair : pt.member) {
                if (bump_pair.first.layer == 1) {
                    bump_pair.first.layer = rc.layer_count - 1;
                }
                if (bump_pair.second.layer == 1) {
                    bump_pair.second.layer = rc.layer_count - 1;
                }
            }
        }
    }
    for (unsigned int i = 0; i < result.team_layer.size(); i++) {
        // std::cout << "PairTeam " << (i + 1) << " assigned to layer " << result.team_layer[i] << std::endl;
        // std::cout << "start group:" << std::endl;
        // std::cout << "left bound x: " << vpt[i].start_left_bound_x << ", right bound x: " << vpt[i].start_right_bound_x
        //          << ", up bound y: " << vpt[i].start_up_bound_y << ", down bound y: " << vpt[i].start_down_bound_y << std::endl;
        // std::cout << "end group:" << std::endl;
        // std::cout << "left bound x: " << vpt[i].end_left_bound_x << ", right bound x: " << vpt[i].end_right_bound_x
        //          << ", up bound y: " << vpt[i].end_up_bound_y << ", down bound y: " << vpt[i].end_down_bound_y << std::endl;
    }
    const std::string output_filename = "output/" + rc.case_name +  "_layer_assignment.txt";
    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
       // std::cerr << "\nError: Failed to open output file " << output_filename << std::endl;
       return result;
    }
    outfile << "PairTeam Layer Assignment Info" << std::endl;
    outfile << "---" << std::endl;
    for (unsigned int i = 0; i < result.team_layer.size(); i++) {
        outfile << "PairTeamID: " << (i + 1) << std::endl;
        outfile << "Layer: " << result.team_layer[i] << std::endl;
        outfile << "StartLeft: " << vpt[i].start_left_bound_x << std::endl;
        outfile << "StartRight: " << vpt[i].start_right_bound_x << std::endl;
        outfile << "StartUp: " << vpt[i].start_up_bound_y << std::endl;
        outfile << "StartDown: " << vpt[i].start_down_bound_y << std::endl;
        outfile << "EndLeft: " << vpt[i].end_left_bound_x << std::endl;
        outfile << "EndRight: " << vpt[i].end_right_bound_x << std::endl;
        outfile << "EndUp: " << vpt[i].end_up_bound_y << std::endl;
        outfile << "EndDown: " << vpt[i].end_down_bound_y << std::endl;
        outfile << "---" << std::endl;
    }
    outfile.close();
    return result;
}