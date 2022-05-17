/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

			pcl::PointXYZ point = cloud->points[i];

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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
