#ifndef LAYER_ASSIGNMENT_H
#define LAYER_ASSIGNMENT_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <climits>
#include <cmath>
#include "kmean.h"
#include "elements.h"

// Point definitions and basic operations
typedef std::pair<int, int> intPoint;
typedef std::pair<double, double> Point;
inline Point operator+(const Point& a, const Point& b) { return {a.first + b.first, a.second + b.second}; }
inline Point operator-(const Point& a, const Point& b) { return {a.first - b.first, a.second - b.second}; }
inline double dot(const Point& a, const Point& b) { return a.first * b.first + a.second * b.second; }
inline double cross(const Point& a, const Point& b) { return a.first * b.second - a.second * b.first; }
inline double length(const Point& a) { return sqrt(dot(a, a)); }
inline Point normalize(const Point& a) { double l = length(a); return l == 0 ? a : Point{a.first/l, a.second/l}; }

// Bus occupancy area definition: Rectangle with cut corners
struct BusOccupancy {
    // 0 is top-left
    int x_min, x_max; // Original rectangle's x-range
    int y_min, y_max; // Original rectangle's y-range
    // Position and size of the cut corners
    std::unordered_map<int, int> corner_cuts; // key=position(1=top-left, 2=top-right, 3=bottom-right, 4=bottom-left), value=size
};

// Result structure for layer assignment
struct LayerAssignmentResult {
    std::vector<int> team_layer;  // Index=PairTeam ID, Value=Assigned layer number (0-based)
    int total_layers;             // Minimum required intermediate routing layers
    bool is_optimal;              // Whether it is a strictly optimal solution
};

BusOccupancy calculate_occupancy(
    int start_left_bound_x,
    int start_right_bound_x,
    int start_up_bound_y,
    int start_down_bound_y,
    int end_left_bound_x,
    int end_right_bound_x,
    int end_up_bound_y,
    int end_down_bound_y
);

std::vector<Point> generate_cut_rect_vertices(const BusOccupancy& occ);

class LayerAssigner {
private:
    std::pair<double, double> project_convex_polygon(const std::vector<Point>& poly, const Point& axis) const;
    bool convex_polygons_intersect(const std::vector<Point>& polyA, const std::vector<Point>& polyB) const;
    bool bus_conflict(const std::vector<Point>& polyA, const std::vector<Point>& polyB) const;
    std::vector<std::vector<int>> build_conflict_graph(const std::vector<PairTeam>& vpt) const;
    void backtrack(
        int current_idx,
        const std::vector<std::vector<int>>& conflict_graph,
        std::vector<int>& current_assignment,
        int current_max_layer,
        std::vector<int>& best_assignment,
        int& best_total_layers
    ) const;
    LayerAssignmentResult assign_optimal_layers(const std::vector<PairTeam>& vpt);
    LayerAssignmentResult assign_approximate_layers(const std::vector<PairTeam>& vpt);

public:
    LayerAssignmentResult assign_layers(std::vector<PairTeam>& vpt, RoutingCase& rc);
};

#endif