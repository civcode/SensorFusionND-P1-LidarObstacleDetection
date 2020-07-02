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
    bool renderScene = false; //true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    //void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    //renderRays(viewer, lidar->position, cloud);
    
    //void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1,1,1));
    //renderPointCloud(viewer, cloud, "some name");
    
    
    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(cloud, 100, 0.2);

    //renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0.7,0.7,0.7));

    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

    

  
}


void cityBlockSingleCloud(pcl::visualization::PCLVisualizer::Ptr& viewer) {

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
   
    // Filter input cloud
    float box_size = 0.25;
    Eigen::Vector4f crop_point_min (-10.0, -6.0, -2.0, 1.0);
    Eigen::Vector4f crop_point_max (30.0, 6.0, 4.0, 1.0);
    //float box_size = 0.1;
    //Eigen::Vector4f crop_point_min (-20.0, -16.0, -2.0, 1.0);
    //Eigen::Vector4f crop_point_max (40.0, 16.0, 4.0, 1.0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, box_size, crop_point_min, crop_point_max);
   
    // Segmentation (RANSAC plane; remaining points)
    int max_iterations = 100;
    float distance_threshold = 0.2; //RANSAC plane segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, max_iterations, distance_threshold);

    //renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    
    // Clustering
    float cluster_tolerance = 0.5;
    int clustering_min_size = 10;
    int clustering_max_size = 1000;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, cluster_tolerance, clustering_min_size, clustering_max_size);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), 
                                  Color(1,1,0), 
                                  Color(0,1,1), 
                                  Color(1,0,1),
                                  Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {

        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        clusterId++;
    }

   
    //renderPointCloud(viewer, inputCloud, "inputCloud", Color(0.7,0.7,0.7));
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    //renderPointCloud(viewer, filterCloud, "inputCloud");
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
   
    // Filter input cloud
    float box_size = 0.25;
    Eigen::Vector4f crop_point_min (-10.0, -6.0, -2.0, 1.0);
    Eigen::Vector4f crop_point_max (30.0, 6.0, 4.0, 1.0);
    //float box_size = 0.1;
    //Eigen::Vector4f crop_point_min (-20.0, -16.0, -2.0, 1.0);
    //Eigen::Vector4f crop_point_max (40.0, 16.0, 4.0, 1.0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, box_size, crop_point_min, crop_point_max);
   
    // Segmentation (RANSAC plane; remaining points)
    int max_iterations = 100;
    float distance_threshold = 0.2; //RANSAC plane segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, max_iterations, distance_threshold);

    //renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    
    // Render ground plane
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,0.7,0));
    
    // Clustering
    float cluster_tolerance = 0.4;
    int clustering_min_size = 8;
    int clustering_max_size = 1000;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, cluster_tolerance, clustering_min_size, clustering_max_size);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), 
                                  Color(1,1,0), 
                                  Color(0,1,1), 
                                  Color(1,0,1),
                                  Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {

        //std::cout << "cluster size ";
        //pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        clusterId++;
    }

   
    //renderPointCloud(viewer, inputCloud, "inputCloud", Color(0.7,0.7,0.7));
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    //renderPointCloud(viewer, filterCloud, "inputCloud");
}


void myCityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
   
    // Filter input cloud
    float box_size = 0.25; //0.25
    Eigen::Vector4f crop_point_min (-10.0, -6.0, -2.0, 1.0);
    Eigen::Vector4f crop_point_max (30.0, 6.0, 4.0, 1.0);
    //float box_size = 0.1;
    //Eigen::Vector4f crop_point_min (-20.0, -16.0, -2.0, 1.0);
    //Eigen::Vector4f crop_point_max (40.0, 16.0, 4.0, 1.0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, box_size, crop_point_min, crop_point_max);
   
    // Segmentation (RANSAC plane; remaining points)
    int max_iterations = 20;
    float distance_threshold = 0.2; //RANSAC plane segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->mySegmentPlane(filterCloud, max_iterations, distance_threshold);

    // Render ground plane
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,0.7,0));
    
    // Clustering
    float cluster_tolerance = 0.4; //0.4
    int clustering_min_size = 8;
    int clustering_max_size = 1000;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->myClustering(segmentCloud.first, cluster_tolerance, clustering_min_size, clustering_max_size);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), 
                                  Color(1,1,0), 
                                  Color(0,1,1), 
                                  Color(1,0,1),
                                  Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {

        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        clusterId++;
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
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    //simpleHighway(viewer);
    //cityBlock(viewer);

    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 

    while (!viewer->wasStopped()) {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        //myCityBlock(viewer, pointProcessorI, inputCloudI);
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}
