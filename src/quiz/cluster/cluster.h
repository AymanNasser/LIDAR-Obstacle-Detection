
#ifndef CLUSTER_H
#define CLUSTER_H

#include <chrono>
#include <string>
#include "kdtree.h"

#include "../../render/render.h"
#include "../../render/box.h"

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom);

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points);

void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth);

void proximity(std::vector<std::vector<float>> &points, KdTree* tree, std::vector<int> &cluster, std::vector<bool> &isProcessed, int pntIndex, float distanceTol);

std::vector<std::vector<int>> euclideanCluster(std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif