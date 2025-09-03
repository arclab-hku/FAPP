#ifndef DBSCAN_H
#define DBSCAN_H

#include <pcl/point_types.h>
#include <nanoflann.hpp>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters(const pcl::PointIndices &a, const pcl::PointIndices &b)
{
    return (a.indices.size() < b.indices.size());
}

template <typename PointT>
class DBSCANSimpleCluster
{

public:
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
    virtual void setInputCloud(PointCloudPtr cloud)
    {
        input_cloud_ = cloud;
    }

       void setSearchMethod(KdTreePtr tree)
    {
             search_method_ = tree;
    }
    void setSearchMethod()
    {
        pcPtr_ = std::make_shared<Obs>();
        // The adaptor
        // cout << "get dyn"<<endl;
        pcPtr_->pts = input_cloud_;
        // cout << "get cloud:\n"<<cloud->size()<<endl;
        kdPtr_ = std::make_shared<my_kd_tree_t>(3, *pcPtr_);
        // cout << "build index"<<endl;
        kdPtr_->buildIndex();
    }

void extractNano(std::vector<pcl::PointIndices> &cluster_indices)
    {
        // std::vector<int> nn_indices;
        // std::vector<float> nn_distances;
         std::vector<std::pair<long unsigned int, double>> ret_matches;

        std::vector<bool> is_noise(input_cloud_->points.size(), false);
        std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
        for (size_t i = 0; i < input_cloud_->points.size(); i++)
        {
            if (types[i] == PROCESSED)
            {
                continue;
            }
            size_t nn_size = radiusSearchNano(i, eps_, ret_matches);
            if (nn_size < minPts_)
            {
                is_noise[i] = true;
                continue;
            }

            std::vector<uint32_t> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;

            for (size_t j = 0; j < nn_size; j++)
            {
                if (ret_matches[j].first  != i)
                {
                    seed_queue.push_back(ret_matches[j].first);
                    types[ret_matches[j].first] = PROCESSING;
                }
            } // for every point near the chosen core point.
            size_t sq_idx = 1;
            while (sq_idx < seed_queue.size())
            {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED)
                {
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue; // no need to check neighbors.
                }
                nn_size = radiusSearchNano(cloud_index, eps_, ret_matches);
                if (nn_size >= minPts_)
                {
                    for (size_t j = 0; j < nn_size; j++)
                    {
                        if (types[ret_matches[j].first] == UN_PROCESSED)
                        {

                            seed_queue.push_back(ret_matches[j].first);
                            types[ret_matches[j].first] = PROCESSING;
                        }
                    }
                }

                types[cloud_index] = PROCESSED;
                sq_idx++;
            }
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size() <= max_pts_per_cluster_)
            {
                pcl::PointIndices r;
                r.indices.resize(seed_queue.size());
                for (size_t j = 0; j < seed_queue.size(); ++j)
                {
                    r.indices[j] = seed_queue[j];
                }
                // These two lines should not be needed: (can anyone confirm?) -FF
                std::sort(r.indices.begin(), r.indices.end());
                r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

                r.header = input_cloud_->header;
                cluster_indices.push_back(r); // We could avoid a copy by working directly in the vector
            }
        } // for every point in input cloud
        std::sort(cluster_indices.rbegin(), cluster_indices.rend(), comparePointClusters);
    }

    // void extract(std::vector<pcl::PointIndices> &cluster_indices)
    // {
    //     std::vector<int> nn_indices;
    //     std::vector<float> nn_distances;
    //     std::vector<bool> is_noise(input_cloud_->points.size(), false);
    //     std::vector<int> types(input_cloud_->points.size(), UN_PROCESSED);
    //     for (size_t i = 0; i < input_cloud_->points.size(); i++)
    //     {
    //         if (types[i] == PROCESSED)
    //         {
    //             continue;
    //         }
    //         int nn_size = radiusSearch(i, eps_, nn_indices, nn_distances);
    //         if (nn_size < minPts_)
    //         {
    //             is_noise[i] = true;
    //             continue;
    //         }

    //         std::vector<int> seed_queue;
    //         seed_queue.push_back(i);
    //         types[i] = PROCESSED;

    //         for (int j = 0; j < nn_size; j++)
    //         {
    //             if (nn_indices[j] != i)
    //             {
    //                 seed_queue.push_back(nn_indices[j]);
    //                 types[nn_indices[j]] = PROCESSING;
    //             }
    //         } // for every point near the chosen core point.
    //         size_t sq_idx = 1;
    //         while (sq_idx < seed_queue.size())
    //         {
    //             int cloud_index = seed_queue[sq_idx];
    //             if (is_noise[cloud_index] || types[cloud_index] == PROCESSED)
    //             {
    //                 // seed_queue.push_back(cloud_index);
    //                 types[cloud_index] = PROCESSED;
    //                 sq_idx++;
    //                 continue; // no need to check neighbors.
    //             }
    //             nn_size = radiusSearch(cloud_index, eps_, nn_indices, nn_distances);
    //             if (nn_size >= minPts_)
    //             {
    //                 for (int j = 0; j < nn_size; j++)
    //                 {
    //                     if (types[nn_indices[j]] == UN_PROCESSED)
    //                     {

    //                         seed_queue.push_back(nn_indices[j]);
    //                         types[nn_indices[j]] = PROCESSING;
    //                     }
    //                 }
    //             }

    //             types[cloud_index] = PROCESSED;
    //             sq_idx++;
    //         }
    //         if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size() <= max_pts_per_cluster_)
    //         {
    //             pcl::PointIndices r;
    //             r.indices.resize(seed_queue.size());
    //             for (int j = 0; j < seed_queue.size(); ++j)
    //             {
    //                 r.indices[j] = seed_queue[j];
    //             }
    //             // These two lines should not be needed: (can anyone confirm?) -FF
    //             std::sort(r.indices.begin(), r.indices.end());
    //             r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

    //             r.header = input_cloud_->header;
    //             cluster_indices.push_back(r); // We could avoid a copy by working directly in the vector
    //         }
    //     } // for every point in input cloud
    //     std::sort(cluster_indices.rbegin(), cluster_indices.rend(), comparePointClusters);
    // }

    void setClusterTolerance(double tolerance)
    {
        eps_ = tolerance;
    }

    void setMinClusterSize(int min_cluster_size)
    {
        min_pts_per_cluster_ = min_cluster_size;
    }

    void setMaxClusterSize(int max_cluster_size)
    {
        max_pts_per_cluster_ = max_cluster_size;
    }

    void setCorePointMinPts(int core_point_min_pts)
    {
        minPts_ = core_point_min_pts;
    }

protected:
    PointCloudPtr input_cloud_;

    double eps_{0.0};
    size_t minPts_{1}; // not including the point itself.
    size_t min_pts_per_cluster_{1};
    size_t max_pts_per_cluster_{std::numeric_limits<int>::max()};

    KdTreePtr search_method_;

    size_t radiusSearchNano(
        int index, double radius, std::vector<std::pair<long unsigned int, double>>& ret_matches)
        {
        check_pt = {input_cloud_->points[index].x,input_cloud_->points[index].y,input_cloud_->points[index].z};
        //  size_t n =  
       return kdPtr_->radiusSearch(
            check_pt.data(), radius, ret_matches, params);
         

        }

    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        k_indices.clear();
        k_sqr_distances.clear();
        k_indices.push_back(index);
        k_sqr_distances.push_back(0);
        int size = input_cloud_->points.size();
        double radius_square = radius * radius;
        for (int i = 0; i < size; i++)
        {
            if (i == index)
            {
                continue;
            }
            double distance_x = input_cloud_->points[i].x - input_cloud_->points[index].x;
            double distance_y = input_cloud_->points[i].y - input_cloud_->points[index].y;
            double distance_z = input_cloud_->points[i].z - input_cloud_->points[index].z;
            double distance_square = distance_x * distance_x + distance_y * distance_y + distance_z * distance_z;
            if (distance_square <= radius_square)
            {
                k_indices.push_back(i);
                k_sqr_distances.push_back(std::sqrt(distance_square));
            }
        }
        return k_indices.size();
    }
private:
    std::vector<double> check_pt;
    struct Obs
    {

        // sensor_msgs::PointCloud *pts;
        PointCloudPtr pts;

        // Must return the number of data points
        // inline size_t kdtree_get_point_count() const { return pts->points.size(); }
        inline size_t kdtree_get_point_count() const { return pts->points.size(); }

        inline double kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            if (dim == 0)
                return pts->points[idx].x;
            else if (dim == 1)
                return pts->points[idx].y;
            else
                return pts->points[idx].z;
        }
        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
    };

    // Obs pc2kd_;
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, Obs>,
        Obs, 3 /* dim */>
        my_kd_tree_t;
    // my_kd_tree_t index(3 /*dim*/, pc2kd_, nanoflann::KDTreeSingleIndexAdaptorParams(20 /* max leaf */));
    std::shared_ptr<Obs> pcPtr_;
    std::shared_ptr<my_kd_tree_t> kdPtr_;
    nanoflann::SearchParams params;
}; // class DBSCANCluster

#endif // DBSCAN_H