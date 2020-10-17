/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "cluster.h"

void proximity(std::vector<std::vector<float>> &points, KdTree* tree, std::vector<int> &cluster, std::vector<bool> &isProcessed, int pntIndex, float distanceTol)
{
	isProcessed[pntIndex] = true;
	cluster.push_back(pntIndex);

	std::vector<int> nearbyPntIds = tree->search(points[pntIndex], distanceTol);

	for(int id : nearbyPntIds){

		if(!isProcessed[id])
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
