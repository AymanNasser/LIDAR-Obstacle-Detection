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
#define SEG_THRESHOLD 0.4

// Clustering macros
#define CLUSTER_MIN_SIZE 10
#define CLUSTER_MAX_SIZE 3000
#define CLUSTER_TOLERANCE 0.5

// Filtering macros
#define VOXEL_GRID_SIZE 0.25

void lidarObstacleDetection(pcl::visualization::PCLVisualizer::Ptr& viewer,
                            ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud )
{



    // Basic stream rendering
    //renderPointCloud(viewer, inputCloud, "Filterd Cloud");

    // Filtering the cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = \
        pointCloudProcessor->FilterCloud(inputCloud, VOXEL_GRID_SIZE, 
                                    Eigen::Vector4f (-10,-5,-2,1),
                                    Eigen::Vector4f (30,8,10,1) ); 

    // Segmenting the cloud to obstacles & plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = \
    pointCloudProcessor->SegmentPlane(filterCloud, SEG_MAX_ITER, SEG_THRESHOLD);

    // Clustering obstacles object
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>  cloudClusters = \ 
    pointCloudProcessor->Clustering(segmentCloud.first,CLUSTER_TOLERANCE, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);


    std::vector<Color> colors = {Color(1,1,1), Color(0,1,0), Color(1,1,0)};
    short colorId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: cloudClusters){

        // std::cout << "Cluster size: " << processPntCld.numPoints(cluster) << std::endl;
        renderPointCloud(viewer, cluster, "Cluster " + std::to_string(colorId), colors[colorId]);
        // Rendering a box 
        Box box = pointCloudProcessor->BoundingBox(cluster);
        renderBox(viewer,box,colorId);
        colorId = (colorId + 1) % 3;
    }


    // renderPointCloud(viewer, segmentCloud.first, "Obstacles Rendering", Color(0,1,0));
    // renderPointCloud(viewer, segmentCloud.second, "Plane Rendering", Color(1,0,0));


}

void test(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = \ 
        pointCloudProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");


    // Filtering the cloud
    /* pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = \
        pointCloudProcessor->FilterCloud(inputCloud, VOXEL_GRID_SIZE, 
                                    Eigen::Vector4f (-10,-5,-2,1),
                                    Eigen::Vector4f (30,8,1,1) ); 
 */
    // Segmenting the cloud to obstacles & plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = \
    pointCloudProcessor->SegmentPlane(inputCloud, SEG_MAX_ITER, SEG_THRESHOLD);


    renderPointCloud(viewer, segmentCloud.first, "Obstacles Rendering", Color(0,0,1));
    renderPointCloud(viewer, segmentCloud.second, "Plane Rendering", Color(1,0,0));



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

    // Testing via single point cloud frame
    // test(viewer);

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