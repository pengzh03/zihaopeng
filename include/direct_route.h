#ifndef INC_2_5D_ROUTING_DIRECT_ROUTE_H
#define INC_2_5D_ROUTING_DIRECT_ROUTE_H

#include "elements.h"
#include "kmean.h"
#include "geometry.h"
#include <climits>
#include <algorithm>
#include <vector>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

struct DirectRoutingResult {
    DRResultDict dr_result_dict;
    std::vector<std::string> final_failed_nets;
};

enum ExitPos {
    UP, LEFT, RIGHT, DOWN
};

enum RouteType {
    FromStart, FromEnd
};

DirectRoutingResult direct_route(const std::vector<PairTeam>& vpt, RoutingCase& rc);

std::string get_node_key_2(float x, float y, int layer);

struct AStarNodeReroute {
    int x, y, layer;
    int g;
    int h;
    int f;
    AStarNodeReroute* parent;

    AStarNodeReroute(int x_, int y_, int layer_);

    bool operator>(const AStarNodeReroute& other) const;
};

struct AStarResultReroute {
    std::vector<AStarNodeReroute> path;
    std::unordered_set<AStarNodeReroute*> nodes_to_delete;
};

#endif //INC_2_5D_ROUTING_DIRECT_ROUTE_H
