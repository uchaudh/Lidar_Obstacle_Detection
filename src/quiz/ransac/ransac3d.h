#ifndef RANSAC_3D_H_
#define RANSAC_3D_H_


#include <unordered_set>

template<typename PointT>
class RANSAC3D {
public:

    std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

};

#endif