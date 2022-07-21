/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool render_clusters = true;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    //Create lidar sensor
    Lidar* lidar= new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud= lidar->scan();
    // renderRays(viewer,lidar->position,pointcloud);
    // renderPointCloud(viewer,pointcloud,"inputcloud");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentcloud = pointProcessor->SegmentPlane(pointcloud,100,0.2);
    renderPointCloud(viewer,segmentcloud.first,"obstaclecloud",Color(1,0,0));
    renderPointCloud(viewer,segmentcloud.second,"planecloud",Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentcloud.first,1.0,4.0,30.0);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstacleCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstacleCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        }
        if(render_box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }
        ++clusterId;
    }

    renderPointCloud(viewer,segmentcloud.second,"planeCloud");

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor->FilterCloud(inputCloud, 0.1 , Eigen::Vector4f (-20, -6, -3, 5), Eigen::Vector4f ( 25, 6.5, 3, 1));

    /* Obstacle detection */
    // Distinguish between road and obstacles and remove road plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessor->SegmentPlane(filterCloud, 25, 0.2);

    // Perform clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters = pointProcessor->Clustering(segment_cloud.first, 0.4, 30, 5000);

    renderPointCloud(viewer, segment_cloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segment_cloud.second, "planeCloud", Color(0, 1, 0));

    // Bounding boxes around obstacles
    int cluster_id = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloud_clusters) 
    {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(cluster_id),colors[cluster_id % colors.size()]);

        // Use standard bounding box
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, cluster_id);
        ++cluster_id;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting environment" << std::endl;

    std::string data;

    if (argc != 2) {
        data = "../src/sensors/data/pcd/data_1";
    } 
    else {
        std::string ds(argv[1]);
        if (ds != "data_1" && ds != "data_2") {
            std::cout << "Choose one of 'data_1' or 'data_2'" << std::endl;
            return 1;
        }
        data = "../src/sensors/data/pcd/" + ds;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(data);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    while (!viewer->wasStopped ())
    {

      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;

      if(streamIterator == stream.end())
        streamIterator = stream.begin();
    
        viewer->spinOnce ();
    } 
}