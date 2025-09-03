#ifndef DBSCAN_H
#define DBSCAN_H


#include <vector>

namespace dbscan {

/** cluster
 *   Does density-based spatial clustering.
 *
 * @param points
 *   Un-t clustered points.
 *
 * @param epsilon
 *   Epsilon hyper-parameter.
 *
 * @param minPoints
 *   Number of points in epsilon neighourhood
 *
 * @return
 *   Clustered indexes.
 */
std::vector<std::vector<unsigned long>> cluster(
    std::vector<Point>& points, const float& epsilon, const int& minPoints);
}
#endif /* DBSCAN_H */
