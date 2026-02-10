#ifndef A_STAR_H
#define A_STAR_H

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <climits>
#include <algorithm>
#include "json.hpp"
#include "elements.h"
#include "kmean.h"
#include "layer_assignment.h"

using json = nlohmann::json;

struct AStarNode {
    int x, y, layer;
    int g;
    int h;
    int f;
    AStarNode* parent;

    AStarNode(int x_, int y_, int layer_);

    bool operator>(const AStarNode& other) const;
};

struct AStarResult {
    std::vector<AStarNode> path;
    std::unordered_set<AStarNode*> nodes_to_delete;
};

struct EscapeOutput {
    std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> escape_results;
    std::unordered_map<int, std::vector<std::pair<size_t, int>>> all_fail_bumps;
};

std::string get_coord_node_key(int x, int y, int layer);

std::string get_node_key(float x, float y, int layer);

int grid_distance(int x1, int y1, int x2, int y2);

std::vector<AStarNode> get_neighbors(const AStarNode& current, int max_width, int max_height, int min_width, int min_height, const std::unordered_set<std::string>& occupied_set, int end_x, int end_y, int end_layer);

bool is_point_in_convex_polygon(double x, double y, const std::vector<Point>& edges);

std::vector<AStarNode> get_neighbors_intersect(const AStarNode& current, const std::vector<Point>& edges, const std::unordered_set<std::string>& occupied_set, int end_x, int end_y, int end_layer);

std::vector<AStarNode> reconstruct_path(AStarNode* end_node, std::unordered_set<std::string>& occupied_set);

AStarResult a_star_find_path_to_exit(
    const Bump& bump,
    int max_width,
    int max_height,
    int min_width,
    int min_height,
    std::unordered_set<std::string>& occupied_set
);

AStarResult a_star_find_path_to_exit_intersect(
    const Bump& bump,
    const std::vector<Point>& edges,
    std::unordered_set<std::string>& occupied_set
);

bool is_same_direction(int dx1, int dy1, int dl1, int dx2, int dy2, int dl2);

std::vector<std::pair<ResultPoint, ResultPoint>> split_to_segments(const std::vector<AStarNode>& path);

EscapeOutput a_star_escape(
    RoutingCase& rc,
    const std::vector<PairTeam>& vpt,
    std::vector<bool> intersect_flags
);

EscapeOutput a_star_escape_multithread(
        RoutingCase& rc,
        const std::vector<PairTeam>& vpt,
        std::vector<bool> intersect_flags
);

#endif