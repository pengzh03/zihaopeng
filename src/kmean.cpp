#include "kmean.h"
#include <fstream>
#include <random>

void KmeanSolver::init_dist() {
    // IOUtils::print("Initing Kmeans Properties...");
    start_bump_dist.assign(num_bumps, std::vector<double>(num_bumps));
    end_bump_dist.assign(num_bumps, std::vector<double>(num_bumps));
    start_bumps_sorted_indices.assign(num_bumps, std::vector<int>(num_bumps));
    end_bumps_sorted_indices.assign(num_bumps, std::vector<int>(num_bumps));
    for (size_t i = 0; i < num_bumps; i++) {
        for (size_t j = i + 1; j < num_bumps; j++) {
            const BumpPair &bp1 = bumpPairs[i];
            const BumpPair &bp2 = bumpPairs[j];
            start_bump_dist[i][j] = start_bump_dist[j][i] = getVectorLength(
                    PDD{bp1.first.grid_coord_x-bp2.first.grid_coord_x, bp1.first.grid_coord_y-bp2.first.grid_coord_y});
            end_bump_dist[i][j] = end_bump_dist[j][i] = getVectorLength(
                    PDD{bp1.second.grid_coord_x-bp2.second.grid_coord_x, bp1.second.grid_coord_y-bp2.second.grid_coord_y});
        }
    }
    for (size_t i = 0; i < num_bumps; i++) {
        start_bumps_sorted_indices[i] = getDescendingIndices(start_bump_dist[i]);
        end_bumps_sorted_indices[i] = getDescendingIndices(end_bump_dist[i]);
        bp2idx[bumpPairs[i]] = i;
    }
    for (const auto & bumpPair : bumpPairs) {
        BumpPairRad.push_back(unitVec2Rad(getUnitVector(getBumpPairVector(bumpPair))));
    }
}

bool KmeanSolver::canConmbineGroup(const KmeanGroup &kg1, const KmeanGroup &kg2, bool judge_k) const {
    if (judge_k) {
        for (const BumpPair & bp1 : kg1.member) {
            for (const BumpPair & bp2 : kg2.member) {
                if (isIntersect(bp1, bp2)) return false;
            }
        }
    }
    if (kg1.member.size() < min_num_group || kg2.member.size() < min_num_group) return true;
    double min_start_groups_dist = INF, min_end_groups_dist = INF;
    for (const BumpPair & bp1 : kg1.member) {
        size_t bp1_idx = bp2idx.find(bp1)->second;
        for (const BumpPair &bp2: kg2.member) {
            size_t bp2_idx = bp2idx.find(bp2)->second;
            min_start_groups_dist = std::min(min_start_groups_dist, start_bump_dist[bp1_idx][bp2_idx]);
            min_end_groups_dist = std::min(min_end_groups_dist, end_bump_dist[bp1_idx][bp2_idx]);
        }
    }
    double start_dist_kg1 = 0, start_dist_kg2 = 0;
    double end_dist_kg1 = 0, end_dist_kg2 = 0;
    std::unordered_set<size_t> sub_idx_1;
    for (const BumpPair & bp : kg1.member) sub_idx_1.insert(bp2idx.find(bp)->second);
    for (const BumpPair & bp : kg1.member) {
        size_t idx = bp2idx.find(bp)->second;
        for (int i = num_bumps-2; i >= 0; i--) {
            size_t target_idx = start_bumps_sorted_indices[idx][i];
            if (sub_idx_1.find(target_idx) != sub_idx_1.end()) {
                start_dist_kg1 = std::max(start_dist_kg1, start_bump_dist[idx][target_idx]);
                break;
            }
        }
        for (int i = num_bumps-2; i >= 0; i--) {
            size_t target_idx = end_bumps_sorted_indices[idx][i];
            if (sub_idx_1.find(target_idx) != sub_idx_1.end()) {
                end_dist_kg1 = std::max(end_dist_kg1, end_bump_dist[idx][target_idx]);
                break;
            }
        }
    }

    std::unordered_set<size_t> sub_idx_2;
    for (const BumpPair & bp : kg2.member) sub_idx_2.insert(bp2idx.find(bp)->second);
    for (const BumpPair & bp : kg2.member) {
        size_t idx = bp2idx.find(bp)->second;
        for (int i = num_bumps-2; i >= 0; i--) {
            size_t target_idx = start_bumps_sorted_indices[idx][i];
            if (sub_idx_2.find(target_idx) != sub_idx_2.end()) {
                start_dist_kg2 = std::max(start_dist_kg2, start_bump_dist[idx][target_idx]);
                break;
            }
        }
        for (int i = num_bumps-2; i >= 0; i--) {
            size_t target_idx = end_bumps_sorted_indices[idx][i];
            if (sub_idx_2.find(target_idx) != sub_idx_2.end()) {
                end_dist_kg2 = std::max(end_dist_kg2, end_bump_dist[idx][target_idx]);
                break;
            }
        }
    }

    if (min_start_groups_dist > std::max(start_dist_kg1, start_dist_kg2) * max_group_dist_rate
        || min_end_groups_dist > std::max(end_dist_kg1, end_dist_kg2) * max_group_dist_rate){
        return false;
    }
    return true;
}

int get_index(double rad, int num_class) {
    return std::floor(rad * num_class / (2 * M_PI));
}

std::vector<KmeanGroup> KmeanSolver::sortDotsByUnitV() const {
    std::vector<std::vector<int>> bumpClassesIdx(groups);
    for (size_t i = 0; i < BumpPairRad.size(); i++) {
        bumpClassesIdx[get_index(BumpPairRad[i], groups)].push_back(i);
    }
    std::vector<KmeanGroup> vkg;
    int num_zero = INF;
    for (int i = 0, cur_group = -1; i < groups; i++) {
        // IOUtils::print(std::to_string(bumpClassesIdx[i].size()) + " ");
        if (!bumpClassesIdx[i].empty() && num_zero >= max_zeros) {
            vkg.emplace_back();
            cur_group++;
            num_zero = 0;
        } else if (!bumpClassesIdx[i].empty() && num_zero < max_zeros) {
            num_zero = 0;
        } else if (bumpClassesIdx[i].empty() && num_zero < max_zeros) {
            num_zero++;
        }
        for (int idx : bumpClassesIdx[i]) vkg[cur_group].member.push_back(bumpPairs[idx]);
    }
    // IOUtils::print("\n");
//    const std::vector<int> &lastBumpClassesIdx = bumpClassesIdx[bumpClassesIdx.size()-1];
//    if (!bumpClassesIdx[0].empty() && !lastBumpClassesIdx.empty()) {
//        vkg[0].member.insert(vkg[0].member.end(), vkg[vkg.size()-1].member.begin(), vkg[vkg.size()-1].member.end());
//        vkg.pop_back();
//    }
    // IOUtils::print("Final k classes: " + std::to_string(vkg.size()) +".\n");
    std::vector<KmeanGroup> n_vkg;
    for (const KmeanGroup & kg : vkg) {
        if (n_vkg.empty() || !canConmbineGroup(n_vkg.back(), kg, true)) n_vkg.push_back(kg);
        else
            n_vkg.back().member.insert(n_vkg.back().member.end(), kg.member.begin(), kg.member.end());
    }
    if (n_vkg.size() > 1 && canConmbineGroup(n_vkg.back(), n_vkg[0], true)) {
        n_vkg[0].member.insert(n_vkg[0].member.end(), n_vkg.back().member.begin(), n_vkg.back().member.end());
        n_vkg.pop_back();
    }
    for (KmeanGroup kg: n_vkg) {
        kg.update();
    }
    // IOUtils::print("Final Cutted k classes: " + std::to_string(n_vkg.size()) +".\n");
    return n_vkg;
}

size_t KmeanSolver::find_next_farest_bump_idx(const std::vector<std::vector<int>>& indices, const std::unordered_set<size_t> &sub_idx, const std::unordered_set<size_t>& refer_idx) const {
    std::unordered_map<size_t, std::unordered_set<int>> recorded_bumps;
    for (size_t i : refer_idx) recorded_bumps[i] = {};
    for (size_t i = 0; i < num_bumps; i++) {
        for (size_t j : refer_idx) {
            int idx = indices[j][i];
            recorded_bumps[j].insert(idx);
            bool flag = true;
            for (size_t k : refer_idx) {
                if (recorded_bumps[k].find(idx) == recorded_bumps[k].end()) {
                    flag = false;
                    break;
                }
            }
            if (flag && refer_idx.find(idx) == refer_idx.end() && sub_idx.find(idx) != sub_idx.end()) {
                return idx;
            }
        }
    }
    return INF;
}

double getBumpsCostByDist(const std::vector<std::vector<double>> &dist, const std::unordered_set<int> &refer_bumps) {
    size_t num_bumps = dist.size();
    double total_cost = 0;
    for (size_t i = 0; i < num_bumps; i++) {
        double min_cost = INF;
        for (int idx : refer_bumps) {
            if (dist[i][idx] < min_cost) {
                min_cost = dist[i][idx];
            }
        }
        total_cost += min_cost;
    }
    return total_cost / num_bumps;
}

std::vector<KmeanGroup> KmeanSolver::sortDotsBySEP(const std::vector<KmeanGroup> &vkg) const {
    std::vector<KmeanGroup> fvkg;
    // int iter = 0;
    for (const KmeanGroup &kg: vkg) {
        std::unordered_set<size_t> sub_idx;
        for (const BumpPair &bp : kg.member) sub_idx.insert(bp2idx.find(bp)->second);
        bool running = true;
        if (kg.member.size() < 2) running = false;
        std::vector<std::vector<KmeanGroup>> result_dict = {{kg}};
        std::unordered_set<size_t> refer_idx;
        refer_idx.insert(bp2idx.find(kg.member[0])->second);
        while (running && refer_idx.size() < sub_idx.size()) {
            size_t n_s_idx = find_next_farest_bump_idx(start_bumps_sorted_indices, sub_idx, refer_idx);
            size_t n_e_idx = find_next_farest_bump_idx(end_bumps_sorted_indices, sub_idx, refer_idx);
            double s_costs = 0;
            double e_costs = 0;
            for (size_t idx : refer_idx) {
                s_costs += start_bump_dist[n_s_idx][idx];
                e_costs += end_bump_dist[n_e_idx][idx];
            }
            if (s_costs >= e_costs) refer_idx.insert(n_s_idx);
            else refer_idx.insert(n_e_idx);

            std::vector<KmeanGroup> classes;
            for (size_t idx : refer_idx) {
                KmeanGroup temp_kg;
                temp_kg.aver_start_point = {bumpPairs[idx].first.grid_coord_x, bumpPairs[idx].first.grid_coord_y};
                temp_kg.aver_end_point = {bumpPairs[idx].second.grid_coord_x, bumpPairs[idx].second.grid_coord_y};
                classes.push_back(temp_kg);
            }

            // Kmeans Algorithm
            double total_score = INF;
            int kmeans_tole_step = 0;
            while (true) {
                double cur_score = 0;
                for (const BumpPair &bp: kg.member) {
                    KmeanGroup *addKG;
                    double min_score = INF;
                    for (KmeanGroup &ckg: classes) {
                        double s_score = getVectorLength(
                                PDD{ckg.aver_start_point.first-bp.first.grid_coord_x, ckg.aver_start_point.second-bp.first.grid_coord_y});
                        double e_score = getVectorLength(
                                PDD{ckg.aver_end_point.first-bp.second.grid_coord_x, ckg.aver_end_point.second-bp.second.grid_coord_y});
                        double score = s_score + e_score;
                        if (min_score > score) {
                            min_score = score;
                            addKG = &ckg;
                        }
                    }
                    cur_score += min_score;
                    addKG->member.push_back(bp);
                }

//                // IOUtils::print("The score of Kmeans Step " + std::to_string(++step) + " : " + std::to_string(cur_score) + ".\n");
                for (KmeanGroup &ckg: classes) ckg.update();
                if (total_score <= cur_score + EPS && ++kmeans_tole_step > 2) break;
                else {
                    total_score = cur_score;
                    for (KmeanGroup &ckg: classes) {
                        ckg.member.clear();
                    }
                }
            }
            for (size_t i = 0; i < classes.size(); i++) {
                for (size_t j = i + 1; j < classes.size(); j++) {
                    if (canConmbineGroup(classes[i], classes[j], false)) {
                        running = false;
                        break;
                    }
                }
                if (!running) break;
            }
            if (running) result_dict.push_back(classes);
        }
        fvkg.insert(fvkg.end(), result_dict.back().begin(), result_dict.back().end());
        // IOUtils::print("Dealing the " + std::to_string(++iter) + " group, the group add another " + std::to_string(result_dict.back().size()-1) + " groups.\n");
    }
    return fvkg;
}

std::vector<PairTeam> KmeanSolver::sortDots() {
    // IOUtils::print("Starting kmeans algorithm...\n");
    std::vector<PairTeam> pt;
    for (const KmeanGroup& kg : sortDotsBySEP(sortDotsBySEP(sortDotsByUnitV()))) {
        pt.emplace_back(kg);
    }
    for (PairTeam& pt_item : pt) {
        pt_item.update_bounds();
    }
    // IOUtils::print("Final classes is: " + std::to_string(pt.size()) + ".\n");
    // if (infile) writeToFile(pt, "output/kmeans_result.txt");
    // IOUtils::print("------Kmeans Algorithm Finished----\n");
    return pt;
}

std::vector<PairTeam> KmeanSolver::sortDotsByCombinedScore() {
    // IOUtils::print("Starting kmeans algorithm...\n");
    const std::unordered_set<int> indices = getRandomIndices(0, bumpPairs.size()-1, init_group_num);
    std::vector<KmeanGroup> vkg;
    for (int i : indices) {
        BumpPair bp = bumpPairs[i];
        vkg.push_back(KmeanGroup{getUnitVector(getBumpPairVector(bp)),
                                 PDD{bp.first.grid_coord_x, bp.first.grid_coord_y},
                                 PDD{bp.second.grid_coord_x, bp.second.grid_coord_y},
                                 std::vector<BumpPair>{},
                                 std::vector<PDD>{}});
    }

    double total_score = INF;
    int tole_step = 0;
    // int step = 0;
    while (true) {
        double cur_score = 0;
        for (const BumpPair &bp: bumpPairs) {
            PDD bp_unit = getUnitVector(getBumpPairVector(bp));
            KmeanGroup *addKG;
            double min_score = INF;
            for (KmeanGroup &kg: vkg) {
                double k_score = dot_product(bp_unit, kg.aver_unit_vec);
                double s_score = getVectorLength(
                        PDD{kg.aver_start_point.first-bp.first.grid_coord_x, kg.aver_start_point.second-bp.first.grid_coord_y});
                double e_score = getVectorLength(
                        PDD{kg.aver_end_point.first-bp.second.grid_coord_x, kg.aver_end_point.second-bp.second.grid_coord_y});
                double score = -k_score + s_score + e_score;
                if (min_score > score) {
                    min_score = score;
                    addKG = &kg;
                }
            }
            cur_score += min_score;
            addKG->member.push_back(bp);
            addKG->unit_member.push_back(bp_unit);
        }

        // IOUtils::print("The score of Kmeans Step " + std::to_string(++step) + " : " + std::to_string(cur_score) + ".\n");
        for (KmeanGroup &kg: vkg) kg.update();
        if (total_score <= cur_score + EPS && ++tole_step > 2) break;
        else {
            total_score = cur_score;
            for (KmeanGroup &kg: vkg) {
                kg.member.clear();
                kg.unit_member.clear();
            }
        }
    }

    std::vector<PairTeam> pt;
    for (const KmeanGroup& kg : vkg) {
        pt.emplace_back(kg);
    }
    for (PairTeam& pt_item : pt) {
        pt_item.update_bounds();
    }
    // if (infile) writeToFile(pt, "output/kmeans_result.txt");
    // IOUtils::print("------Kmeans Algorithm Finished----\n");
    return pt;
}

void KmeanSolver::writeToFile(const std::vector<PairTeam> &vpt, const std::string& file_path)  {
    std::ofstream ofs(file_path);

    if (!ofs.is_open()) {
        IOUtils::terminateProgram("Failed to open file: "+file_path);
    }

    for (size_t i = 1; i <= vpt.size(); i++) {
        ofs << i << std::endl;
        for (const BumpPair &bp : vpt[i-1].member) {
            ofs << "[[" << bp.first.grid_coord_x << "," << bp.first.grid_coord_y << "," << bp.first.layer << "],";
            ofs << "[" << bp.second.grid_coord_x << "," << bp.second.grid_coord_y << "," << bp.second.layer << "]]\t";
        }
        ofs << std::endl;
    }
    ofs << "----" << std::endl;
    for (size_t i = 1; i <= vpt.size(); i++) {
        ofs << i << std::endl;
        ofs << "[[" << vpt[i-1].start_up_bound_y << "," << vpt[i-1].start_right_bound_x << "," << vpt[i-1].start_down_bound_y << "," << vpt[i-1].start_left_bound_x << "],["
        << vpt[i-1].end_up_bound_y << "," << vpt[i-1].end_right_bound_x << "," << vpt[i-1].end_down_bound_y << "," << vpt[i-1].end_left_bound_x << "]]" << std::endl;
    }

    // IOUtils::print("Saved to Output file "+file_path+"\n");
    ofs.close();
}