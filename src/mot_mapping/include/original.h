#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "point.h"

// original implementation of dbscan
// algorithm provided on wiki:
// see@: https://en.wikipedia.org/wiki/DBSCAN
//
namespace original {

const int FAIL = -3;
const int PASS = 0;

std::vector<int> queryRange(std::shared_ptr<std::vector<Point>>& sptr_points,
    const Point& core, const float& epsilon)
{
    int index = 0;
    std::vector<int> neighbours;
    for (const auto& point : *sptr_points) {
        if (core.distance(point) <= epsilon) {
            neighbours.push_back(index);
        }
        index++;
    }
    return neighbours;
}

int search(std::shared_ptr<std::vector<Point>>& sptr_points, Point point,
    int cluster, const int& N, const float& E)
{
    std::vector<int> neighbours = queryRange(sptr_points, point, E);
    if (neighbours.size() < N) {
        point.m_cluster = NOISE;
        return FAIL;
    }

    int index = 0;
    int core = 0;
    for (auto neighbour : neighbours) {
        sptr_points->at(neighbour).m_cluster = cluster;
        if (sptr_points->at(neighbour) == point) {
            core = index;
        }
        ++index;
    }
    neighbours.erase(neighbours.begin() + core);
    for (std::vector<int>::size_type i = 0, n = neighbours.size(); i < n; ++i) {
        std::vector<int> nextSet
            = queryRange(sptr_points, sptr_points->at(neighbours[i]), E);

        if (nextSet.size() >= N) {
            for (auto neighbour : nextSet) {
                if (sptr_points->at(neighbour).unlabeled()) {
                    neighbours.push_back(neighbour);
                    n = neighbours.size();
                    sptr_points->at(neighbour).m_cluster = cluster;
                }
            }
        }
    }
    return PASS;
}

int cluster(std::shared_ptr<std::vector<Point>>& sptr_points, const int& N,
    const float& E)
{
    int cluster = 0;
    for (const auto& point : *sptr_points) {
        if (point.m_cluster == UNLABELED) {
            if (search(sptr_points, point, cluster, N, E) != FAIL) {
                cluster += 1;
            }
        }
    }
    return cluster;
}
}
