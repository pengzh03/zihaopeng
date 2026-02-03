#ifndef IOUTILS_H
#define IOUTILS_H

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include "elements.h"
#include "json.hpp"

using json = nlohmann::json;

struct RunParams {
    std::string case_name = "test";
    bool thread = true;
    int min_rate = 2;
    int max_zero = 2;
    std::string grid_layout_path;
    std::string netlist_path;
    std::string output_path;
};

RunParams parseCmd(int argc, char *argv[]);
void checkParams(const RunParams& rp);

class IOUtils {
public:
    static void terminateProgram(const std::string& errorMessage);
    static void print(const std::string& message);
    static void printParams(const RunParams& rp);
};

json coord_to_json(int x, int y, int layer, int layer_count);

void output_routing_result(
    const RoutingCase& rc,
    const std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>>& routing_paths,
    const std::string& output_filename
);

#endif