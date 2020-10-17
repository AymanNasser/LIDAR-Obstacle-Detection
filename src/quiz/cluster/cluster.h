
#ifndef CLUSTER_H
#define CLUSTER_H

#include <chrono>
#include <string>
#include "kdtree.h"


void proximity(std::vector<std::vector<float>> &points, KdTree* tree, std::vector<int> &cluster, std::vector<bool> &isProcessed, int pntIndex, float distanceTol);

std::vector<std::vector<int>> euclideanCluster(std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif