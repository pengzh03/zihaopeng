#pragma once
#include <string>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#define _USE_MATH_DEFINES
#include <cmath>

// struct TupleHash {
//     template <typename T1, typename T2, typename T3>
//     size_t operator()(const std::tuple<T1, T2, T3>& t) const {
//         size_t h1 = std::hash<T1>{}(std::get<0>(t));
//         size_t h2 = std::hash<T2>{}(std::get<1>(t));
//         size_t h3 = std::hash<T3>{}(std::get<2>(t));
//         return h1 ^ (h2 << 1) ^ (h3 << 2);
//     }
// };

struct ResultPoint {
    int x;
    int y;
    int layer;
};

//Start point to End point
typedef std::pair<ResultPoint, ResultPoint> Seg;

//Net name to seg line
//{
//    "Net1" : {
//        {
//            start_seg,
//            end_seg,
//        }
//        ...
//    }
//    ...
//}
typedef std::unordered_map<std::string, std::vector<Seg>> DRResultDict, ResultDict;

struct ExitTerminal {
    std::string bump_name;
    int grid_coord_x;
    int grid_coord_y;
    int layer;
    bool operator==(const ExitTerminal& other) const {
        return grid_coord_x == other.grid_coord_x
               && grid_coord_y == other.grid_coord_y
               && layer == other.layer;
    }
};

struct Bump {
    std::string net_name;
    int bump_type;
    int grid_coord_x;
    int grid_coord_y;
    int layer;
    ExitTerminal exit_terminal;
    bool operator==(const Bump& other) const {
        return net_name == other.net_name
               && bump_type == other.bump_type
               && grid_coord_x == other.grid_coord_x
               && grid_coord_y == other.grid_coord_y
               && layer == other.layer;
    }
};

struct Net {
    int net_type;
    int bump_count;
    std::vector<std::string> bump_names;
};

struct RoutingCase {
    std::string case_name;
    int grid_length;
    int grid_max_width;
    int grid_max_height;
    int layer_count;
    std::unordered_map<std::string, Bump> bump_dict;
    int net_count;
    std::vector<std::string> net_name_list;
    std::unordered_map<std::string, Net> net_dict;
    std::unordered_set<std::string> occupied_set;
    std::unordered_set<std::string> occupied_set_2d;
};