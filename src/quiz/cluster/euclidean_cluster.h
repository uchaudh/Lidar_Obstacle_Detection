/* \author Utkarsha Chaudhari */

#ifndef __EUCLIDEAN_CLUSTER_H__
#define __EUCLIDEAN_CLUSTER_H__

#include "kdtree.h"


void euclideanclusterHelper(int i,const std::vector<std::vector<float>> points,std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree, float distanceTol);

std::vector<std::vector<int>> euclidean_Cluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);


#endif /* __EUCLIDEAN_CLUSTER_H__ */