/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <string>

// Segmentation macros
#define SEG_MAX_ITER 100
#define SEG_THRESHOLD 0.35

// Clustering macros
#define CLUSTER_MIN_SIZE 10
#define CLUSTER_MAX_SIZE 3000
#define CLUSTER_TOLERANCE 0.4

// Filtering macros
#define VOXEL_GRID_SIZE 0.15

// Color macros
#define BLUE Color(0,0,1)
#define WHITE Color(1,1,1)
#define RED Color(1,0,0)
#define GREEN Color(0,1,0)
#define MIXED_1 Color(1,1,0)
#define MIXED_2 Color(0,1,1)
#define MIXED_3 Color(1,0,1)

#define TEST 0

void lidarObstacleDetection(pcl::visualization::PCLVisualizer::Ptr& viewer,
                            ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud )
{

    Eigen::Vector4f minPoint (-30, -6.5, -3, 1);
    Eigen::Vector4f maxPoint (30, 6.5, 8, 1);

    // Filtering the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterdCloud = \
    pointCloudProcessor->FilterCloud(inputCloud, VOXEL_GRID_SIZE, minPoint, maxPoint);

    // Segmenting the cloud to obstacles & plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = \
    pointCloudProcessor->SegmentPlane(filterdCloud, SEG_MAX_ITER, SEG_THRESHOLD);

    renderPointCloud(viewer, segmentCloud.first, "Obstacles Rendering", BLUE);
    renderPointCloud(viewer, segmentCloud.second, "Plane Rendering", RED);

    // Clustering obstacles object
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>  cloudClusters = \ 
    pointCloudProcessor->Clustering(segmentCloud.first,CLUSTER_TOLERANCE, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);


    std::vector<Color> colors = {MIXED_1, MIXED_2, MIXED_3};
    uint clusterId = 0;
    float boxOpacity = 0.8;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters){

        // std::cout << "Cluster size: " << processPntCld.numPoints(cluster) << std::endl;
        renderPointCloud(viewer, cluster, "Cluster " + std::to_string(clusterId), colors[clusterId%3]);
        // Rendering a box 
        // Box box = pointCloudProcessor->BoundingBox(cluster);
        BoxQ box = pointCloudProcessor->BoundingBoxQ(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId%3], boxOpacity);
        clusterId++;
    }

}

void test(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = \ 
        pointCloudProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");


    Eigen::Vector4f minPoint (-30, -6.5, -3, 1);
    Eigen::Vector4f maxPoint (30, 6.5, 8, 1);

    // Filtering the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterdCloud = \
    pointCloudProcessor->FilterCloud(inputCloud, VOXEL_GRID_SIZE, minPoint, maxPoint);

    // Segmenting the cloud to obstacles & plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = \
    pointCloudProcessor->SegmentPlane(filterdCloud, SEG_MAX_ITER, SEG_THRESHOLD);

    renderPointCloud(viewer, segmentCloud.first, "Obstacles Rendering", BLUE);
    renderPointCloud(viewer, segmentCloud.second, "Plane Rendering", RED);

    // Clustering obstacles object
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>  cloudClusters = \ 
    pointCloudProcessor->Clustering(segmentCloud.first,CLUSTER_TOLERANCE, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);


    std::vector<Color> colors = {MIXED_1, MIXED_2, MIXED_3};
    int clusterId = 0;
    float boxOpacity = 1;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters){

        // std::cout << "Cluster size: " << processPntCld.numPoints(cluster) << std::endl;
        renderPointCloud(viewer, cluster, "Cluster " + std::to_string(clusterId), colors[clusterId%3]);
        // Rendering a box 
        Box box = pointCloudProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId%3], boxOpacity);
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

    // Loading data stream 
    std::string dataPath = "../src/sensors/data/pcd/data_1";
    ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointCloudProcessor->streamPcd(dataPath);
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    
    if(TEST)
    {   
        // Testing via single point cloud frame
        test(viewer);
        while( !viewer->wasStopped() )
            viewer->spinOnce();    
    }

    else
    {
        while (!viewer->wasStopped ()){
        
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointCloudProcessor->loadPcd((*streamIterator).string());
        lidarObstacleDetection(viewer, pointCloudProcessor, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }    
    }
    


}