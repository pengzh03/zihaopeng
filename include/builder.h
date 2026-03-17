#ifndef BUILDER_H
#define BUILDER_H

#include "elements.h"
#include "json.hpp"
#include <vector>

typedef std::pair<Bump, Bump> BumpPair;

template <class T>
inline void hash_combine(std::size_t& seed, const T& val) {
    std::hash<T> hasher;
    seed ^= hasher(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
    template<> struct hash<Bump> {
        size_t operator()(const Bump& bump) const {
            size_t seed = 0;
            // 依次混合Bump的所有成员的哈希值
            hash_combine(seed, bump.net_name);      // string的哈希由std::hash<string>处理
            hash_combine(seed, bump.bump_type);     // int的哈希由std::hash<int>处理
            hash_combine(seed, bump.grid_coord_x);
            hash_combine(seed, bump.grid_coord_y);
            hash_combine(seed, bump.layer);
            return seed; // 最终哈希值
        }
    };

    template<> struct hash<BumpPair> {
        size_t operator()(const BumpPair& pair) const {
            size_t seed = 0;
            hash_combine(seed, pair.first);
            hash_combine(seed, pair.second);
            return seed;
        }
    };
}


using json = nlohmann::json;

json load_json(const std::string& path);
RoutingCase build_routing_case(const std::string& case_name, const std::string& layout_path, const std::string& netlist_path);
// void showcase(const RoutingCase& routing_case);
std::vector<RoutingCase> load_case(std::string case_name, std::string grid_layout_path, std::string netlist_path);
std::vector<BumpPair> split_multi_pin_net(const std::vector<Bump>& bumps);
std::vector<BumpPair> get_all_bump_pair(const RoutingCase &rc);

#endif
