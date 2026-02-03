#include <iostream>
#include <fstream>
#include "builder.h"
#include "ioutils.h"

const std::string TESTCASE_NAMES[6] =
{
    "C2IO1",
    "C8IO1",
    "C4M4",
    "G1M4",
    "TEST",
    "test"
};

json load_json(const std::string& path) {
    std::ifstream fin(path);
    if (!fin.is_open()) {
        // std::cerr << "ERROR: Cannot open file: " + path + ".\n";
        std::abort();
    }
    json object_json;
    fin >> object_json;
    return object_json;
}

RoutingCase build_routing_case(const std::string& case_name, const std::string& layout_path, const std::string& netlist_path) {
    RoutingCase routing_case;
    routing_case.case_name = case_name;

    json layout = load_json(layout_path);
    json netlist = load_json(netlist_path);

    // grid_info
    routing_case.grid_length = layout["grid_info"]["grid_length"];
    routing_case.grid_max_width = layout["grid_info"]["grid_max_width"];
    routing_case.grid_max_height = layout["grid_info"]["grid_max_height"];
    routing_case.layer_count = 2;

    // bottom_layer
    for (auto& raw_bump : layout["bottom_layer"]) {
        Bump new_bump;
        new_bump.net_name = ""; // filled from netlist later
        new_bump.bump_type = 0; // filled from netlist later
        new_bump.grid_coord_x = raw_bump["grid_coord_x"];
        new_bump.grid_coord_y = raw_bump["grid_coord_y"];
        new_bump.layer = 0;
        std::string bump_name = raw_bump["c4_name"];
        routing_case.bump_dict[bump_name] = new_bump;
        std::string coord_key = std::to_string(new_bump.grid_coord_x) + "_" + std::to_string(new_bump.grid_coord_y);
        if (routing_case.occupied_set_2d.find(coord_key) != routing_case.occupied_set_2d.end()) {
            // std::cerr << "Error: Duplicate bump coordinate at (" << new_bump.grid_coord_x << ", " << new_bump.grid_coord_y << ") in bottom layer.\n";
            std::abort();
        }
        routing_case.occupied_set_2d.insert(coord_key);
    }

    // top_layer
    for (auto& raw_bump : layout["top_layer"]) {
        Bump new_bump;
        new_bump.net_name = ""; // filled from netlist later
        new_bump.bump_type = 0; // filled from netlist later
        new_bump.grid_coord_x = raw_bump["grid_coord_x"];
        new_bump.grid_coord_y = raw_bump["grid_coord_y"];
        new_bump.layer = 1;
        std::string bump_name = raw_bump["bump_name"];
        routing_case.bump_dict[bump_name] = new_bump;
        std::string coord_key = std::to_string(new_bump.grid_coord_x) + "_" + std::to_string(new_bump.grid_coord_y);
        if (routing_case.occupied_set_2d.find(coord_key) != routing_case.occupied_set_2d.end()) {
            // std::cerr << "Error: Duplicate bump coordinate at (" << new_bump.grid_coord_x << ", " << new_bump.grid_coord_y << ") in top layer.\n";
            std::abort();
        }
        routing_case.occupied_set_2d.insert(coord_key);
    }

    // netlist
    routing_case.net_count = netlist["net_count"];
    for (auto& raw_net : netlist["nets"]) {
        std::string net_name = raw_net["net_name"];
        routing_case.net_name_list.push_back(net_name);
        Net new_net;
        new_net.net_type = raw_net["net_type"];
        new_net.bump_count = raw_net["bump_count"];
        for (auto& raw_net_bump : raw_net["bumps"]) {
            std::string bump_name = raw_net_bump["bump_name"];
            int bump_type = raw_net_bump["bump_type"];
            new_net.bump_names.push_back(bump_name);
            if (routing_case.bump_dict.count(bump_name)) {
                routing_case.bump_dict[bump_name].net_name = net_name;
                routing_case.bump_dict[bump_name].bump_type = bump_type;
            } else {
                // std::cerr << "Error: bump " << bump_name << " not found in layout.\n";
            }
        }
        routing_case.net_dict[net_name] = new_net;
    }

    return routing_case;
}

// test
// void showcase(const RoutingCase& routing_case) {
//     // std::cout << "=== Routing Case: " << routing_case.case_name << " ===\n\n";

//     // std::cout << "Grid: " << routing_case.grid_length << "\n";
//     // std::cout << "Max Width: " << routing_case.grid_max_width << "\n";
//     // std::cout << "Max Height: " << routing_case.grid_max_height << "\n";
//     // std::cout << "Layers: " << routing_case.layer_count << "\n";
//     // std::cout << "Net Count: " << routing_case.net_count << "\n\n";

//     // std::cout << "[Sample Bumps]\n";
//     // int count = 0;
//     // for (const auto& bump_item : routing_case.bump_dict) {
//     //     const auto& bump = bump_item.second;
//     //     // std::cout << bump_item.first << " -> (x=" << bump.grid_coord_x << ", y=" << bump.grid_coord_y << ", layer=" << bump.layer << ", type=" << bump.bump_type << ", net=" << bump.net_name << ")\n";
//     //     if (++count >= 5) break;
//     // }

//     // std::cout << "\n[Sample Nets]\n";
//     // count = 0;
//     // for (const auto& net_item : routing_case.net_dict) {
//     //     const auto& n = net_item.second;
//     //     // std::cout << net_item.first << " -> type=" << n.net_type << ", bump_count=" << n.bump_count << "\n";
//     //     for (auto& name : n.bump_names)
//     //         // std::cout << "   " << name << "\n";
//     //     if (++count >= 5) break;
//     // }
// }

std::vector<RoutingCase> load_case(std::string case_name, std::string grid_layout_path, std::string netlist_path)
{
    std::vector<RoutingCase> vrc;
    RoutingCase routing_case = build_routing_case(
            case_name,
            grid_layout_path,
            netlist_path
    );
    // showcase(routing_case);
    vrc.push_back(routing_case);
    if (!vrc.size())
    {
        IOUtils::print("Error loading case. '" + case_name + "' does not exist in case list:\n[");
        for (std::string s : TESTCASE_NAMES)
            IOUtils::print(s + ", ");
        IOUtils::terminateProgram("].");
    }
    return vrc;
}

std::vector<BumpPair> split_multi_pin_net(const std::vector<Bump>& bumps) {
    std::vector<std::pair<Bump, Bump>> wire_pairs;
    if (bumps.size() == 2) {
        wire_pairs.emplace_back(bumps[0], bumps[1]);
        return wire_pairs;
    } else {
        // std::cerr << "Error: Net has less or more than 2 bumps.\n";
        std::abort();
    }
}

std::vector<BumpPair> get_all_bump_pair(const RoutingCase &rc) {
    std::vector<BumpPair> vbp;
    for (const std::string& net_name : rc.net_name_list) {
        // for (int iter_i = 0; iter_i < 5; iter_i++) {
        //     const std::string& net_name = rc.net_name_list[iter_i];
        const Net &net = rc.net_dict.at(net_name);
        std::vector<Bump> net_bumps;
        for (const std::string &bump_name: net.bump_names) {
            net_bumps.push_back(rc.bump_dict.at(bump_name));
        }
        std::vector<BumpPair> wire_pairs = split_multi_pin_net(net_bumps);
        vbp.insert(vbp.end(), wire_pairs.begin(), wire_pairs.end());
    }
    return vbp;
}