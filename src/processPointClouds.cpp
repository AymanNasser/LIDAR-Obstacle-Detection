// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filterCloud (new pcl::PointCloud<PointT>());
    // Filtering our point cloud using voxel grid filter
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes,filterRes,filterRes); // Cube dimensions
    sor.filter (*filterCloud);

    // Downsampling cloud with only points that were inside the region specified
    typename pcl::PointCloud<PointT>::Ptr regionCloud (new pcl::PointCloud<PointT>());
    std::vector<int> indicesToRemoved;

    pcl::CropBox<PointT> region(true); // Extract_removed_indices Set to true to extract indicies to be removed
    region.setInputCloud(filterCloud);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.filter(indicesToRemoved);

    // Removing filterd points from output pointcloud
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int index : indicesToRemoved)
        inliers->indices.push_back(index);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filterCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*regionCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> \ 
ProcessPointClouds<PointT>::SeparateClouds(typename pcl::PointCloud<PointT>::Ptr cloud, std::unordered_set<int> inliersResult) 
{

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> \
    segResult(cloudInliers, cloudOutliers);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

    std::unordered_set<int> inliersResult;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    int cloudSize = cloud->points.size();
	while(maxIterations--){
		
		std::unordered_set<int> inliers;

		int randIndex_1 = rand() %  cloudSize;
		int randIndex_2 = rand() % cloudSize;
		int randIndex_3 = rand() % cloudSize;

		pcl::PointXYZ point1 = cloud->points[randIndex_1];
		pcl::PointXYZ point2 = cloud->points[randIndex_2];
		pcl::PointXYZ point3 = cloud->points[randIndex_3];

		float i = (point2.y -point1.y)*(point3.z - point1.z) - 
				  (point2.z - point1.z)*(point3.y - point1.y);

		float j = (point2.z - point1.z)*(point3.x - point1.x) - 
				  (point2.x - point1.x)*(point3.z - point1.z);

		float k = (point2.x - point1.x)*(point3.y - point1.y) - 
				  (point2.y - point1.y)*(point3.x - point1.x);


		for(int pointIndex=0; pointIndex < cloudSize; pointIndex++){
			
			float point_x = cloud->points[pointIndex].x;
			float point_y = cloud->points[pointIndex].y;
			float point_z = cloud->points[pointIndex].z;

			float planeFormula = i*point_x + j*point_y + k*point_z - 
								 (i*point1.x + j*point1.y + k*point1.z);

			float distance = fabs(i*point_x + j*point_y + k*point_z + (i*point1.x + j*point1.y + k*point1.z)) \ 
							/ sqrt(i*i + j*j + k*k);


			if( distance <= distanceThreshold)
				inliers.insert(pointIndex);
		}
		/* Checking if the no. of inliers are bigger the current best result & if so
		*  we update the best result ==> best fitted model
		*/
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}




    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;

    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);    

    for(pcl::PointIndices getIndices : clusterIndices){
        
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index: getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}