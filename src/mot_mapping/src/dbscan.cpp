#include <vector>

#include "dbscan.h"
#include "kdtree.h"
#include "point.h"

std::vector<std::vector<unsigned long>> dbscan::cluster(
    std::vector<Point>& points, const float& epsilon, const int& minPoints)
{
    std::vector<std::vector<unsigned long>> clusters
        = kdtree::cluster(points, epsilon, minPoints);
    return clusters;
}
