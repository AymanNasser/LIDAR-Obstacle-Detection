/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "cluster.h"

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}



void proximity(std::vector<std::vector<float>> &points, KdTree* tree, std::vector<int> &cluster, std::vector<bool> &isProcessed, int pntIndex, float distanceTol)
{
	isProcessed[pntIndex] = true;
	cluster.push_back(pntIndex);

	std::vector<int> nearbyPntIds = tree->search(points[pntIndex], distanceTol);

	for(int id : nearbyPntIds){

		if(isProcessed[pntIndex])
			continue;
		
		proximity(points, tree ,cluster, isProcessed, id, distanceTol);

	}
}

std::vector<std::vector<int>> euclideanCluster(std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	std::vector<std::vector<int>> clusters;
	std::vector<bool> isProcessed(points.size(),false);
	
	for(int pntIndex = 0; pntIndex < points.size(); pntIndex++){

		if(isProcessed[pntIndex])
			continue;
		
		std::vector<int> cluster;
		proximity(points, tree ,cluster, isProcessed, pntIndex, distanceTol);

		clusters.push_back(cluster);
	}

	return clusters;
}
