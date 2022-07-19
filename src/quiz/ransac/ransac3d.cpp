#include <pcl/common/common.h>
#include <unordered_set>

#include <chrono>
#include "../../render/render.h"
#include "ransac3d.h"

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime= std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--)
	{
		//initialise required variables
		float x1,y1,z1,x2,y2,z2,x3,y3,z3;
		float A,B,C,D,d;

		//pick three random points
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}

		//get three points
		auto itr = inliers.begin();

		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// consider point 1 as the reference and create the vectors
		std::tuple<float,float,float> v1(x2-x1,y2-y1,z2-z1);
		std::tuple<float,float,float> v2(x3-x1,y3-y1,z3-z1);

		//create the normal of the two vectors
		std::tuple<float,float,float> normal((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1),
											(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1),
											(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));

		
		//get the slopes
		A = std::get<0>(normal);
		B = std::get<1>(normal);
		C = std::get<2>(normal);
		D = -(A*x1 + B*y1 + C*z1);

		for(int i = 0; i < cloud->points.size(); i++)
		{
			if(inliers.count(i)>0)
			{
				continue;
			}

			const auto& point = cloud->points[i];

			d = fabs(A*point.x + B*point.y + C*point.z + D)/sqrt(A*A + B*B + C*C);

			if(d <= distanceTol)
			{
				inliers.insert(i);
			}
		}

		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	auto endTime= std::chrono::steady_clock::now();
    auto elapseTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout<< "RANSAC Algorthm took: "<< elapseTime.count()<<"microseconds"<<std::endl;
	
	return inliersResult;

}