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
#define SEG_MAX_ITER 30
#define SEG_THRESHOLD 0.35

// Clustering macros
#define CLUSTER_MIN_SIZE 10
#define CLUSTER_MAX_SIZE 500
#define CLUSTER_TOLERANCE 0.53

// Filtering macros
#define VOXEL_GRID_SIZE 0.4

void lidarObstacleDetection(pcl::visualization::PCLVisualizer::Ptr& viewer,
                            ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessor,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud )
{
   // Filtering the cloud
   pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = \
        pointCloudProcessor->FilterCloud(inputCloud, VOXEL_GRID_SIZE, 
                                    Eigen::Vector4f (-10,-5,-2,1),
                                    Eigen::Vector4f (30,8,1,1) ); 

    // Segmenting the cloud to obstacles & plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = \
    pointCloudProcessor->SegmentPlane(inputCloud, SEG_MAX_ITER, SEG_THRESHOLD);


    renderPointCloud(viewer, segmentCloud, "Basic Stream Render");



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