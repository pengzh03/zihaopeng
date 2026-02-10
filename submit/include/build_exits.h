#ifndef BUILD_EXITS_H
#define BUILD_EXITS_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <tuple>
#include "layer_assignment.h"
#include "kmean.h"
#include "elements.h"


enum class EdgeType {
    NONE,
    TOP,
    BOTTOM,
    LEFT,
    RIGHT
};

namespace std {
    template<>
    struct hash<EdgeType> {
        size_t operator()(const EdgeType &e) const {
            // 核心：将enum class转换为底层整数类型（int）后哈希
            return hash<int>()(static_cast<int>(e));
        }
    };
}

struct IntersectionInfo {
    EdgeType edge_type; // which edge the intersection occurs
    int x; // x coordinate of the intersection point
    int y; // y coordinate of the intersection point
    size_t pair_idx; // index of the bump pair in PairTeam.member
    const Bump* a_bump; // bump in the start group
    const Bump* b_bump; // bump in the end group
};

struct AGroupProjection {
    int projection;
    size_t pair_idx;
    bool operator<(const AGroupProjection& other) const {
        if (projection != other.projection) {
            return projection < other.projection;
        }
        return pair_idx < other.pair_idx;
    }
};


class ExitBuilder {
public:
    std::vector<bool> build_exit_terminals( // build exit terminals for all PairTeams
        std::vector<PairTeam>& vpt,
        RoutingCase& rc,
        const LayerAssignmentResult& lar
    );

private:
    template <typename T>
    T clamp(const T& value, const T& min_val, const T& max_val) const;

    // calculate the intersection of a line segment and a rectangle
    IntersectionInfo calc_segment_rect_intersection(
        const int x1, const int y1, const int x2, const int y2,
        const int left, const int right, const int top, const int bottom
    ) const;

    // sort intersection points of a group in clockwise order
    void sort_a_intersections(std::vector<IntersectionInfo>& intersections, const int left, const int right, const int top, const int bottom) const;
    
    // return edge priority for handling b intersections
    std::vector<EdgeType> b_edge_prio(std::vector<IntersectionInfo>& intersections, const int left, const int right, const int top, const int bottom) const;

    // assign uniform positions along the specified edge (maintaining clockwise order)
    std::vector<std::pair<int, int>> assign_uniform_positions(
        const EdgeType edge_type,
        const int left, const int right, const int top, const int bottom,
        const size_t pin_count
    ) const;

    // print debug information
    // void print_debug_info(const std::string& msg) const;
};

#endif