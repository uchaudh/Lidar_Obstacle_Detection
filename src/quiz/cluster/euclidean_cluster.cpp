/* \author Utkarsha Chaudhari */

#include "euclidean_cluster.h"

void euclideanclusterHelper(int i,const std::vector<std::vector<float>> points,std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	//mark index as processed
	processed[i] = true;
	cluster.push_back(i);

	//check which points are near the current index
	std::vector<int> nearest = tree->search(points[i],distanceTol);

	//recusively search for nearerst points
	for(int id : nearest)
	{
		if(!processed[id])
			euclideanclusterHelper(id,points,cluster,processed,tree,distanceTol);
	}
}

std::vector<std::vector<int>> euclidean_Cluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed_points(points.size(),false);

	int i = 0;

	for (int i=0; i<points.size(); ++i)
	{
		if(processed_points[i])
			continue;

		std::vector<int> cluster;
		euclideanclusterHelper(i,points,cluster,processed_points, tree, distanceTol);
		clusters.push_back(cluster);
	}

 
	return clusters;

}