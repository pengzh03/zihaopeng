#include "geometry.h"
#include <cmath>
#include <random>

PII getBumpVector(const Bump &start, const Bump &end) {
    return PII{end.grid_coord_x-start.grid_coord_x, end.grid_coord_y-start.grid_coord_y};
}

PII getBumpPairVector(const BumpPair &bp) {
    return getBumpVector(bp.first, bp.second);
}

double dot_product(const PDD &a, const PDD &b) {
    return a.first * b.first + a.second * b.second;
}

PDD getUnitVector(const PII &vec) {
    double norm = getVectorLength(vec);
    if (norm < EPS) return PDD{0., 0.};
    else return PDD{vec.first/norm, vec.second/norm};
}

double unitVec2Rad(const PDD &unit) {
    double rad = std::atan2(unit.second, unit.first);
    if (rad < 0.0) {
        rad += 2 * M_PI;
    }
    return rad;
}

// 快速排斥实验：判断线段AB和CD的边界框是否重叠
bool quickReject(const Bump& A, const Bump& B, const Bump& C, const Bump& D) {
    // 检查x方向边界框是否重叠
    bool xOverlap = (std::min(A.grid_coord_x, B.grid_coord_x) <= std::max(C.grid_coord_x, D.grid_coord_x)) &&
                    (std::min(C.grid_coord_x, D.grid_coord_x) <= std::max(A.grid_coord_x, B.grid_coord_x));
    // 检查y方向边界框是否重叠
    bool yOverlap = (std::min(A.grid_coord_y, B.grid_coord_y) <= std::max(C.grid_coord_y, D.grid_coord_y)) &&
                    (std::min(C.grid_coord_y, D.grid_coord_y) <= std::max(A.grid_coord_y, B.grid_coord_y));
    return xOverlap && yOverlap; // 两者都重叠则通过
}

double cross(const Bump& p0, const Bump& p1, const Bump& p2) {
    return (p1.grid_coord_x - p0.grid_coord_x) * (p2.grid_coord_y - p0.grid_coord_y) - (p1.grid_coord_y - p0.grid_coord_y) * (p2.grid_coord_x - p0.grid_coord_x);
}

bool onSegment(const Bump& p, const Bump& a, const Bump& b) {
    // p的x坐标在a和b的x范围内，且y坐标在a和b的y范围内
    return (p.grid_coord_x <= std::max(a.grid_coord_x, b.grid_coord_x) && p.grid_coord_x >= std::min(a.grid_coord_x, b.grid_coord_x) &&
            p.grid_coord_y <= std::max(a.grid_coord_y, b.grid_coord_y) && p.grid_coord_y >= std::min(a.grid_coord_y, b.grid_coord_y));
}

bool isIntersect(const BumpPair &bp1, const BumpPair &bp2) {
    const Bump& A = bp1.first;
    const Bump& B = bp1.second;
    const Bump& C = bp2.first;
    const Bump& D = bp2.second;
    // 步骤1：快速排斥实验（不通过则直接不相交）
    if (!quickReject(A, B, C, D)) {
        return false;
    }

    // 步骤2：计算叉积，判断跨立情况
    double c1 = cross(A, B, C); // C相对于AB的位置
    double c2 = cross(A, B, D); // D相对于AB的位置
    double c3 = cross(C, D, A); // A相对于CD的位置
    double c4 = cross(C, D, B); // B相对于CD的位置

    // 情况1：非共线，且互相跨立（c1与c2异号，c3与c4异号）
    if ((c1 * c2 < 0) && (c3 * c4 < 0)) {
        return true;
    }

    // 情况2：共线（叉积为0），需判断是否有端点在线段上
    // C在AB上？
    if (c1 == 0 && onSegment(C, A, B)) return true;
    // D在AB上？
    if (c2 == 0 && onSegment(D, A, B)) return true;
    // A在CD上？
    if (c3 == 0 && onSegment(A, C, D)) return true;
    // B在CD上？
    if (c4 == 0 && onSegment(B, C, D)) return true;

    // 以上情况均不满足，不相交
    return false;
}

std::unordered_set<int> getRandomIndices(int min, int max, size_t size) {
    std::random_device rd;
    std::mt19937 g(rd());
    std::uniform_int_distribution<> dist(min, max);

    std::unordered_set<int> indices;
    while (indices.size() < size) {
        indices.insert(dist(g));
    }
    return indices;
}

std::vector<int> getDescendingIndices(const std::vector<double>& vec) {
    std::vector<int> indices(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        indices[i] = i;
    }
    std::sort(indices.begin(), indices.end(),
              [&vec](int i, int j) {
                  return vec[i] > vec[j];
              });
    return indices;
}
