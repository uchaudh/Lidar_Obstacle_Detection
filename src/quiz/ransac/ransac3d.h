#ifndef _RANSAC_3D_H_
#define _RANSAC_3D_H_


#include <unordered_set>
#include "../../render/render.h"

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);


#endif