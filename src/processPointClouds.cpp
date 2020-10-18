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
    typename pcl::PointCloud<PointT>::Ptr downsampledCloud (new pcl::PointCloud<PointT>());

    // Filtering our point cloud using voxel grid filter
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes,filterRes,filterRes); // Cube dimensions
    sor.filter (*downsampledCloud);

    // Downsampling cloud with only points that were inside the region specified
    typename pcl::PointCloud<PointT>::Ptr filterdCloud (new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> region(false); 
    region.setInputCloud(downsampledCloud);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.filter(*filterdCloud);

    //define a crop-box of apprx size that represents roof points
    std::vector<int> roof_indices;

    pcl::CropBox<PointT> roof;
    region.setInputCloud(filterdCloud);
    region.setMin(Eigen::Vector4f(-1.5,  -1.7, -1, 1));
    region.setMax(Eigen::Vector4f(2.6,  1.7, -0.4, 1));
    region.filter(roof_indices);

    // Removing filterd points from output pointcloud
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int index : roof_indices)
        inliers->indices.push_back(index);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filterdCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filterdCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filterdCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> \ 
ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int>& inliersResult,typename pcl::PointCloud<PointT>::Ptr cloud ) 
{

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> \
    segResult(cloudOutliers, cloudInliers);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, \
     int maxIterations, float distanceThreshold)
{
    
    std::unordered_set<int> inliersResult;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    int cloudSize = cloud->points.size();
	while(maxIterations--){
		
        /* Using unordered_set to avoid picking same points 
        *  So we're looping until the picking 3 diff. point indices & that is when the condition fails
        */
		std::unordered_set<int> inliers;

        while(inliers.size() < 3)
            inliers.insert(rand() %  cloudSize);


        auto itr = inliers.begin();
		PointT point1 = cloud->points[*itr];
        itr++;
		PointT point2 = cloud->points[*itr];
        itr++;
		PointT point3 = cloud->points[*itr];

		float i = (point2.y -point1.y)*(point3.z - point1.z) - 
				  (point2.z - point1.z)*(point3.y - point1.y);

		float j = (point2.z - point1.z)*(point3.x - point1.x) - 
				  (point2.x - point1.x)*(point3.z - point1.z);

		float k = (point2.x - point1.x)*(point3.y - point1.y) - 
				  (point2.y - point1.y)*(point3.x - point1.x);
        
        float d = - (i*point1.x + j*point1.y + k*point1.z);


		for(int pointIndex=0; pointIndex < cloudSize; pointIndex++){
            
            // Checking if the current_point[pointIndex] is one of the 3 picked randomly points &
            // if it was one of them, skip the calculations 
            if (inliers.count(pointIndex) > 0)
                continue;
			
			float point_x = cloud->points[pointIndex].x;
			float point_y = cloud->points[pointIndex].y;
			float point_z = cloud->points[pointIndex].z;

			float planeFormula = i*point_x + j*point_y + k*point_z + d;
								 
			float distance = fabs(planeFormula) / sqrt(i*i + j*j + k*k);

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

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    auto startTime = std::chrono::steady_clock::now();

    KdTree* tree = new KdTree;
    uint cloudSize = cloud->points.size();
    std::vector<std::vector<float>> points;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for(int pntIndex = 0; pntIndex < cloudSize; pntIndex++){
        
        std::vector<float> point_3dVector;
        PointT point = cloud->points[pntIndex];

        point_3dVector.push_back(point.x);
        point_3dVector.push_back(point.y);
        point_3dVector.push_back(point.z);

        tree->insert(point_3dVector, pntIndex);
        points.push_back(point_3dVector);
    }

    std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, clusterTolerance);

    for(std::vector<int> clusterIndex : clusterIndices){
        
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>());

        for(int index: clusterIndex){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        if ((cloudCluster->width >= minSize) && (cloudCluster->width <= maxSize))
            clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " milliseconds" << std::endl;

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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    BoxQ box;
    
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    // The signs are different and the box doesn't get correctly oriented in some cases.
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3> (0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1> (0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    //  quaternion is calculated using the eigenvectors (which determines how the final box gets rotated), and the transform to put the box in correct location is calculated
    const Eigen::Quaternionf bboxQuaternion_(eigenVectorsPCA); 
    const Eigen::Vector3f bboxTransform_ = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // Assigning values to boxQ object
    box.bboxTransform = bboxTransform_;
    box.bboxQuaternion = bboxQuaternion_ ;

    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;
    
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