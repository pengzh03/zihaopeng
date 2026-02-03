#include "ioutils.h"
#include <getopt.h>

struct option long_options[] =
{
    {"case", no_argument, 0, 'c'},
    {"thread", no_argument, 0, 't'},
    {"help", no_argument, 0, 'h'},
    {"max_zeros", no_argument, 0, 'z'},
    {"min_rate", no_argument, 0, 'r'},
    {0, 0, 0, 0}
};

const char* ABBRCOMMAND = "htz:r:c:";
const std::string HELP_INFO =
        std::string("Usage: router <grid_layout_path> <net_list_path> <output_path>\n") +
        "Parameters:\n"
        "  <grid_layout_path>  Path to grid_layout input file (required)\n"
        "  <net_list_path>     Path to net_list input file (required)\n"
        "  <output_path>       Path to output file (required)\n"
        "Note:\n"
        "  - max_zero is fixed to 2\n"
        "  - multi-thread mode is enabled by default\n";

void checkParams(const RunParams& rp)
{
    if (rp.grid_layout_path.empty()) {
        IOUtils::terminateProgram("Error: grid_layout path is empty!");
    }
    if (rp.netlist_path.empty()) {
        IOUtils::terminateProgram("Error: netlist path is empty!");
    }
    if (rp.output_path.empty()) {
        IOUtils::terminateProgram("Error: output path is empty!");
    }
}

RunParams parseCmd(int argc, char *argv[])
{
    RunParams rp;
    rp.case_name = "test";
    rp.max_zero = 2;
    rp.thread = true;

    if (argc != 4) {
        IOUtils::terminateProgram("Error: incorrect number of parameters!\n" + HELP_INFO);
    }

    rp.grid_layout_path = argv[1];
    rp.netlist_path = argv[2]; 
    rp.output_path = argv[3];
    checkParams(rp);

    return rp;
}

void IOUtils::printParams(const RunParams& rp)
{
    IOUtils::print("\n-------------Param Setting-------------");
    IOUtils::print("\nRouting case:\t"+rp.case_name);
    IOUtils::print("\nRouting Thread:\t"+std::to_string(rp.thread));
    IOUtils::print("\nKmeans min rate:\t"+std::to_string(rp.min_rate));
    IOUtils::print("\nKmeans max zero:\t"+std::to_string(rp.max_zero));
    IOUtils::print("\n---------------------------------------\n\n");
}

void IOUtils::terminateProgram(const std::string& errorMessage)
{
    std::cout<< errorMessage <<std::endl;
    exit(1);
}

void IOUtils::print(const std::string& message)
{
    std::cout << message << std::flush;
}

json coord_to_json(int x, int y, int layer, int layer_count) {
    std::string layer_name = (layer == 0) ? "Bottom" : (layer == layer_count - 1) ? "Top" : "M" + std::to_string(layer);
    return json::array({x, y, layer_name});
}

void output_routing_result(
    const RoutingCase& rc,
    const std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>>& routing_paths,
    const std::string& output_filename
) {
    json result;
    for (const std::string& net_name : rc.net_name_list) {
        json net_segments = json::array();
        for (const auto& raw_segment : routing_paths.at(net_name)) {
            json segment;
            segment["start_grid_coordinate"] = coord_to_json(raw_segment.first.x, raw_segment.first.y, raw_segment.first.layer, rc.layer_count);
            segment["end_grid_coordinate"] = coord_to_json(raw_segment.second.x, raw_segment.second.y, raw_segment.second.layer, rc.layer_count);
            net_segments.push_back(segment);
        }
        result[net_name] = net_segments;
    }
    std::string output_file = output_filename;
    std::ofstream ofs(output_file);
    if (ofs.is_open()) {
        ofs << std::setw(4) << result << std::endl;
        std::cout << "\nRouting result saved to: " << output_file << std::endl;
    } else {
        std::cerr << "\nError: Failed to open output file " << output_file << std::endl;
    }
}