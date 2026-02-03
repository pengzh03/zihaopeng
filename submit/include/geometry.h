#ifndef INC_2_5D_ROUTING_GEOMETRY_H
#define INC_2_5D_ROUTING_GEOMETRY_H

#include <iostream>
#include <unordered_set>
#include "elements.h"
#include "builder.h"

typedef std::pair<double, int> PDI;
typedef std::pair<int, int> PII;
typedef std::pair<double, double> PDD;

const double EPS = 1e-4;
const int INF = 1e9;

PII getBumpVector(const Bump &start, const Bump &end);
PII getBumpPairVector(const BumpPair &bp);
PDD getUnitVector(const PII &vec);
double dot_product(const PDD &a, const PDD &b);
double unitVec2Rad(const PDD &unit);

template <typename T>
double getVectorLength(const T &vec) {
    return std::sqrt(vec.first * vec.first + vec.second * vec.second);
}

// 计算叉积 (p1 - p0) × (p2 - p0)
double cross(const Bump& p0, const Bump& p1, const Bump& p2);

// 判断点p是否在线段ab上（需确保p与a、b共线）
bool onSegment(const Bump& p, const Bump& a, const Bump& b);

bool isIntersect(const BumpPair &bp1, const BumpPair &bp2);

std::unordered_set<int> getRandomIndices(int min, int max, size_t size);

std::vector<int> getDescendingIndices(const std::vector<double>& vec);

#endif //INC_2_5D_ROUTING_GEOMETRY_H
