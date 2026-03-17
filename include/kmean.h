#ifndef INC_2_5D_ROUTING_KMEAN_H
#define INC_2_5D_ROUTING_KMEAN_H

#include <utility>

#include "geometry.h"
#include "builder.h"
#include "ioutils.h"

struct KmeanGroup {
    PDD aver_unit_vec, aver_start_point, aver_end_point;
    std::vector<BumpPair> member;
    std::vector<PDD> unit_member;

    void update() {
        update_aver_unit_vec();
        update_aver_start_point();
        update_aver_end_point();
    }

    void update_aver_unit_vec() {
        PDD auv = {0, 0};
        for (PDD um : unit_member) {
            auv.first += um.first;
            auv.second += um.second;
        }
        size_t size = member.size();
        if (size) {
            auv.first /= size;
            auv.second /= size;
        }
        aver_unit_vec = auv;
    }

    void update_aver_start_point() {
        PDD asp = {0, 0};
        for (const BumpPair& m : member) {
            asp.first += m.first.grid_coord_x;
            asp.second += m.first.grid_coord_y;
        }
        size_t size = member.size();
        if (size) {
            asp.first /= size;
            asp.second /= size;
        }
        aver_start_point = asp;
    }

    void update_aver_end_point() {
        PDD aep = {0, 0};
        for (const BumpPair& m : member) {
            aep.first += m.second.grid_coord_x;
            aep.second += m.second.grid_coord_y;
        }
        size_t size = member.size();
        if (size) {
            aep.first /= size;
            aep.second /= size;
        }
        aver_end_point = aep;
    }

};

class PairTeam {
public:
// assume the coordinate of 0 is up and left.
    int start_left_bound_x, start_right_bound_x;
    int end_left_bound_x, end_right_bound_x;
    int start_up_bound_y, start_down_bound_y;
    int end_up_bound_y, end_down_bound_y;
    PDD aver_unit_vector;
    size_t cnt;
    std::vector<BumpPair> member;
    PairTeam (const KmeanGroup& kg) {
        this->member = kg.member;
        this->aver_unit_vector = kg.aver_unit_vec;
        this->cnt = member.size();
        this->start_left_bound_x = this->end_left_bound_x = this->start_up_bound_y = this->end_up_bound_y = INF;
        this->start_right_bound_x = this->end_right_bound_x = this->start_down_bound_y = this->end_down_bound_y = -1;
    }
    void update_bounds() {
        for (const BumpPair& bp : this->member) {
            this->start_left_bound_x = std::min(this->start_left_bound_x, bp.first.grid_coord_x);
            this->start_up_bound_y = std::min(this->start_up_bound_y, bp.first.grid_coord_y);
            this->start_right_bound_x = std::max(this->start_right_bound_x, bp.first.grid_coord_x);
            this->start_down_bound_y = std::max(this->start_down_bound_y, bp.first.grid_coord_y);
            this->end_left_bound_x = std::min(this->end_left_bound_x, bp.second.grid_coord_x);
            this->end_up_bound_y = std::min(this->end_up_bound_y, bp.second.grid_coord_y);
            this->end_right_bound_x = std::max(this->end_right_bound_x, bp.second.grid_coord_x);
            this->end_down_bound_y = std::max(this->end_down_bound_y, bp.second.grid_coord_y);
        }
    }
};

class KmeanSolver {
private:
    int init_group_num = 10;
    int groups = 360;
    int max_zeros = 1;
    double max_group_dist_rate = 2;
    size_t min_num_group = 2;
    bool infile = true;

    size_t num_bumps;

    std::vector<BumpPair> bumpPairs;
    std::vector<std::vector<double>> start_bump_dist;
    std::vector<std::vector<double>> end_bump_dist;
//    from max to min
    std::vector<std::vector<int>> start_bumps_sorted_indices;
    std::vector<std::vector<int>> end_bumps_sorted_indices;
    std::vector<double> BumpPairRad;
    std::unordered_map<BumpPair, size_t> bp2idx;

    void init_dist();
    bool canConmbineGroup(const KmeanGroup &kg1, const KmeanGroup &kg2, bool judge_k) const;
    size_t find_next_farest_bump_idx(const std::vector<std::vector<int>>& indices, const std::unordered_set<size_t> &sub_idx, const std::unordered_set<size_t>& refer_bumps) const;
    void writeToFile(const std::vector<PairTeam> &vpt, const std::string &file_path);
    std::vector<KmeanGroup> sortDotsByUnitV() const;
    std::vector<KmeanGroup> sortDotsBySEP(const std::vector<KmeanGroup> &vkg) const;

public:
    KmeanSolver(int max_zeros, int min_rate, const std::vector<BumpPair> &bumpPairs) {
        this->max_zeros = max_zeros;
        this->max_group_dist_rate = min_rate;
        this->bumpPairs = bumpPairs;
        this->num_bumps = bumpPairs.size();
        init_dist();
    }
    std::vector<PairTeam> sortDots();
    std::vector<PairTeam> sortDotsByCombinedScore();
};


#endif //INC_2_5D_ROUTING_KMEAN_H
