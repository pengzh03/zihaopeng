#include "ioutils.h"
#include "builder.h"
#include "kmean.h"
#include "layer_assignment.h"
#include "build_exits.h"
#include "a_star.h"
#include "direct_route.h"
#include "a_star_direct.h"

// 新增：计时&日志所需头文件
#include <chrono>
#include <fstream>
#include <iomanip>
#include <string>
#include <stdexcept>

// 封装：高精度计时工具（毫秒级，保留3位小数）
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using DurationMs = std::chrono::duration<double, std::milli>;

// 日志文件流（全局，便于各步骤调用）
std::ofstream log_file;

void init_log_file(const std::string& output_path) {
    // 定义需要替换的后缀
    const std::string target_suffix = "_routing.json";
    const std::string log_suffix = "_log.txt";
    std::string log_filename;

    // 查找并替换后缀：_routing.json -> _log.txt
    size_t suffix_pos = output_path.find(target_suffix);
    if (suffix_pos != std::string::npos) {
        // 找到目标后缀，替换为日志后缀
        log_filename = output_path.substr(0, suffix_pos) + log_suffix;
    } else {
        // 未找到目标后缀，兜底处理（末尾追加_log.txt）
        log_filename = output_path + log_suffix;
        std::cerr << "[Warning] Output path does not contain suffix '" << target_suffix 
                  << "', log file set to: " << log_filename << std::endl;
    }

    // 打开日志文件（覆盖模式）
    log_file.open(log_filename, std::ios::out | std::ios::trunc);
    if (!log_file.is_open()) {
        throw std::runtime_error("Failed to open log file: " + log_filename);
    }

    // 写入日志头（补充output_path和log文件路径）
    log_file << "===== Routing Program Timing Log =====" << std::endl;
    log_file << "Output Path: " << output_path << std::endl;
    log_file << "Log File: " << log_filename << std::endl;
    log_file << "Start Time: " << std::chrono::system_clock::to_time_t(Clock::now()) << std::endl;
    log_file << "----------------------------------------" << std::endl;
    log_file << std::fixed << std::setprecision(3); // 耗时保留3位小数
}

// 辅助函数：记录步骤耗时并写入日志
void log_step_time(const std::string& step_name, TimePoint start, TimePoint end) {
    double elapsed_ms = DurationMs(end - start).count();
    double elapsed_s = elapsed_ms / 1000.0;
    log_file << "[STEP] " << step_name << std::endl;
    log_file << "  Elapsed Time: " << elapsed_s << " s" << std::endl;
    log_file << "----------------------------------------" << std::endl;
    // 同时输出到控制台（可选，便于实时查看）
    std::cout << "[Timing] " << step_name << " completed in " << elapsed_s << " s" << std::endl;
}

int main(int argc, char *argv[])
{
    // 全局计时开始
    TimePoint program_start = Clock::now();

    try {
        // STEP 1: Read Parameters.
        TimePoint step1_start = Clock::now();
        RunParams rp = parseCmd(argc, argv);
        TimePoint step1_end = Clock::now();

        // 初始化日志文件（基于case_name）
        init_log_file(rp.output_path);
        log_step_time("1. Read Parameters", step1_start, step1_end);

        // STEP 2: Load Route Information.
        TimePoint step2_start = Clock::now();
        std::vector<RoutingCase> vrc = load_case(rp.case_name, rp.grid_layout_path, rp.netlist_path);
        TimePoint step2_end = Clock::now();
        log_step_time("2. Load Route Information", step2_start, step2_end);

        // STEP 3: Classify Nets
        TimePoint step3_start = Clock::now();
        KmeanSolver ks(rp.max_zero, rp.min_rate, get_all_bump_pair(vrc[0]));
        std::vector<PairTeam> vpt = ks.sortDots();
        TimePoint step3_end = Clock::now();
        log_step_time("3. Classify Nets", step3_start, step3_end);

        // STEP 4: Layer Assignment.
        TimePoint step4_start = Clock::now();
        LayerAssigner la;
        LayerAssignmentResult lar = la.assign_layers(vpt, vrc[0]);
        TimePoint step4_end = Clock::now();
        log_step_time("4. Layer Assignment", step4_start, step4_end);

        // STEP 5: Build Exits.
        TimePoint step5_start = Clock::now();
        ExitBuilder eb;
        std::vector<bool> intersect_flags = eb.build_exit_terminals(vpt, vrc[0], lar);
        TimePoint step5_end = Clock::now();
        log_step_time("5. Build Exits", step5_start, step5_end);

        // STEP 6: Escape Routing.
        TimePoint step6_start = Clock::now();
        EscapeOutput escape_output;
        if (rp.thread) escape_output = a_star_escape_multithread(vrc[0], vpt, intersect_flags);
        else escape_output = a_star_escape(vrc[0], vpt, intersect_flags);
        auto& escape_paths = escape_output.escape_results;
        TimePoint step6_end = Clock::now();
        log_step_time("6. Escape Routing", step6_start, step6_end);

        // STEP 7: Area Routing.
        TimePoint step7_start = Clock::now();
        std::unordered_map<std::string, std::vector<std::pair<ResultPoint, ResultPoint>>> routing_paths;
        auto direct_res = direct_route(vpt, vrc[0]);
        DRResultDict drrd = direct_res.dr_result_dict;
        TimePoint step7_end = Clock::now();
        log_step_time("7. Area Routing", step7_start, step7_end);

        // STEP 8: Rerouting.
        TimePoint step8_start = Clock::now();
        RerouteResult reroute_result;
        if (rp.thread) reroute_result = a_star_reroute_multithread(vrc[0], vpt, escape_output.all_fail_bumps);
        else reroute_result = a_star_reroute(vrc[0], vpt, escape_output.all_fail_bumps);
        auto& reroute_paths = reroute_result.escape_results;
        TimePoint step8_end = Clock::now();
        log_step_time("8. Rerouting", step8_start, step8_end);

        // STEP 9: Recording and Saving Result.
        TimePoint step9_start = Clock::now();
        for (size_t team_idx = 0; team_idx < vpt.size(); ++team_idx) {
            const PairTeam& pt = vpt[team_idx];
            for (const BumpPair& bp : pt.member) {
                const std::string& net_name = bp.first.net_name;
                if (vrc[0].bump_dict.at(vrc[0].net_dict.at(net_name).bump_names[0]).grid_coord_x != bp.first.grid_coord_x ||
                    vrc[0].bump_dict.at(vrc[0].net_dict.at(net_name).bump_names[0]).grid_coord_y != bp.first.grid_coord_y ||
                    vrc[0].bump_dict.at(vrc[0].net_dict.at(net_name).bump_names[0]).layer != bp.first.layer) {
                    const std::string& first_bump_name = vrc[0].net_dict.at(net_name).bump_names[1];
                    const std::string& second_bump_name = vrc[0].net_dict.at(net_name).bump_names[0];
                    routing_paths[net_name] = {};
                    auto& first_bump_escape_path = escape_paths.at(first_bump_name);
                    routing_paths[net_name].insert(
                        routing_paths[net_name].end(),
                        first_bump_escape_path.begin(),
                        first_bump_escape_path.end()
                    );
                    if (reroute_paths.find(first_bump_name) != reroute_paths.end()) {
                        auto& first_bump_reroute_path = reroute_paths.at(first_bump_name);
                        routing_paths[net_name].insert(
                            routing_paths[net_name].end(),
                            first_bump_reroute_path.begin(),
                            first_bump_reroute_path.end()
                        );
                    }
                    routing_paths[net_name].insert(
                        routing_paths[net_name].end(),
                        drrd[net_name].begin(),
                        drrd[net_name].end()
                    );
                    auto& second_bump_escape_path = escape_paths.at(second_bump_name);
                    routing_paths[net_name].insert(
                        routing_paths[net_name].end(),
                        second_bump_escape_path.rbegin(),
                        second_bump_escape_path.rend()
                    );
                    if (reroute_paths.find(second_bump_name) != reroute_paths.end()) {
                        auto& second_bump_reroute_path = reroute_paths.at(second_bump_name);
                        routing_paths[net_name].insert(
                            routing_paths[net_name].end(),
                            second_bump_reroute_path.rbegin(),
                            second_bump_reroute_path.rend()
                        );
                    }
                } else {
                    const std::string& first_bump_name = vrc[0].net_dict.at(net_name).bump_names[0];
                    const std::string& second_bump_name = vrc[0].net_dict.at(net_name).bump_names[1];
                    routing_paths[net_name] = {};
                    auto& first_bump_escape_path = escape_paths.at(first_bump_name);
                    routing_paths[net_name].insert(
                        routing_paths[net_name].end(),
                        first_bump_escape_path.begin(),
                        first_bump_escape_path.end()
                    );
                    if (reroute_paths.find(first_bump_name) != reroute_paths.end()) {
                        auto& first_bump_reroute_path = reroute_paths.at(first_bump_name);
                        routing_paths[net_name].insert(
                            routing_paths[net_name].end(),
                            first_bump_reroute_path.begin(),
                            first_bump_reroute_path.end()
                        );
                    }
                    routing_paths[net_name].insert(
                        routing_paths[net_name].end(),
                        drrd[net_name].begin(),
                        drrd[net_name].end()
                    );
                    auto& second_bump_escape_path = escape_paths.at(second_bump_name);
                    routing_paths[net_name].insert(
                        routing_paths[net_name].end(),
                        second_bump_escape_path.rbegin(),
                        second_bump_escape_path.rend()
                    );
                    if (reroute_paths.find(second_bump_name) != reroute_paths.end()) {
                        auto& second_bump_reroute_path = reroute_paths.at(second_bump_name);
                        routing_paths[net_name].insert(
                            routing_paths[net_name].end(),
                            second_bump_reroute_path.rbegin(),
                            second_bump_reroute_path.rend()
                        );
                    }
                }
            }
        }
        output_routing_result(vrc[0], routing_paths, rp.output_path);
        TimePoint step9_end = Clock::now();
        log_step_time("9. Recording and Saving Result", step9_start, step9_end);

        // 全局计时结束，写入总耗时
        TimePoint program_end = Clock::now();
        double total_ms = DurationMs(program_end - program_start).count();
        double total_s = total_ms / 1000.0;
        double total_min = total_s / 60.0;
        log_file << "[TOTAL] Program Execution Time" << std::endl;
        log_file << "  Elapsed Time: " <<  total_s << " s , " << total_min << " min" << std::endl;
        log_file << "========================================" << std::endl;
        std::cout << "[Timing] Total Program Time: " << total_s << " s (" << total_min << " min)" << std::endl;

        // 关闭日志文件
        if (log_file.is_open()) {
            log_file.close();
        }

    } catch (const std::exception& e) {
        // 异常处理：保证日志文件正常关闭，输出错误信息
        std::cerr << "[ERROR] Program failed: " << e.what() << std::endl;
        if (log_file.is_open()) {
            log_file << "[ERROR] Program failed: " << e.what() << std::endl;
            log_file.close();
        }
        return 1;
    }

    return 0;
}
