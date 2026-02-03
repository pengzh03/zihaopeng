#ifndef A_STAR_DIRECT_H
#define A_STAR_DIRECT_H

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

struct AStarNodeDirect {
    int x, y, layer;
    int g;
    int h;
    int f;
    AStarNodeDirect* parent;

    AStarNodeDirect(int x_, int y_, int layer_);

    bool operator>(const AStarNodeDirect& other) const;
};

struct AStarResultDirect {
    std::vector<AStarNodeDirect> path;
    std::unordered_set<AStarNodeDirect*> nodes_to_delete;
};


struct RerouteResult {
    std::vector<std::string> reroute_fail_nets;
    std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> escape_results;
};

std::string get_coord_node_key_direct(int x, int y, int layer);

std::string get_node_key_direct(float x, float y, int layer);

int grid_distance_direct(int x1, int y1, int l1, int x2, int y2, int l2);

std::vector<AStarNodeDirect> get_neighbors_direct(const AStarNodeDirect& current, int max_width, int max_height, int layer_count, const std::unordered_set<std::string>& occupied_set, int end_x, int end_y, int end_layer);

std::vector<AStarNodeDirect> reconstruct_path_direct(AStarNodeDirect* end_node, std::unordered_set<std::string>& occupied_set);

AStarResultDirect a_star_find_path_direct(const Bump& start, int max_width, int max_height, int layer_count, std::unordered_set<std::string>& occupied_set);

json coord_to_json_direct(int x, int y, int layer);

bool is_same_direction_direct(int dx1, int dy1, int dl1, int dx2, int dy2, int dl2);

std::vector<std::pair<ResultPoint, ResultPoint>> split_to_segments_direct(const std::vector<AStarNodeDirect>& path);

RerouteResult a_star_reroute(
    RoutingCase& rc,
    const std::vector<PairTeam>& vpt,
    std::unordered_map<int, std::vector<std::pair<size_t, int>>> all_fail_bumps
);

RerouteResult a_star_reroute_multithread(
        RoutingCase& rc,
        const std::vector<PairTeam>& vpt,
        std::unordered_map<int, std::vector<std::pair<size_t, int>>> all_fail_bumps
);

#endif