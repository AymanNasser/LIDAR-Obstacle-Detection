/* \author Aaron Brown */
// Quiz on implementing kd tree

// #include "../../render/render.h"
#include <math.h>
#include <vector>

#define DEPTH_MOD 2

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
	uint depth;
	KdTree()
	: root(NULL), depth(0)
	{}


	void auxInsert(Node **node, std::vector<float> point, int id, uint depth)
	{
		if( *node == NULL){
			*node = new Node(point, id);
		}

		else{
			
			if(point[depth%DEPTH_MOD] < (*node)->point[depth%DEPTH_MOD])
				auxInsert(&((*node)->left), point, id, depth+1);	
			else
				auxInsert(&((*node)->right), point, id, depth+1);	

		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		auxInsert(&(this->root), point, id, 0);
	}

	void auxSearch(Node *node, std::vector<float> target, int depth, float &distanceTol, std::vector<int> &ids )
	{
		if(node != NULL)
		{	
			// Checking if the node within the box
			if( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && \
			    (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) )
			   {
				// Checking if the the point within the distance tolerance
				float distance = sqrt( pow(node->point[0] - target[0],2) + pow(node->point[1] - target[1],2) );
				if(distance <= distanceTol)
					ids.push_back(node->id);
			   }

			   	// Whether to split left or right
			   	if( (target[depth%DEPTH_MOD] - distanceTol) < node->point[depth%DEPTH_MOD])
					auxSearch(node->left, target, depth+1, distanceTol, ids);
				if( (target[depth%DEPTH_MOD] + distanceTol) > node->point[depth%DEPTH_MOD])
					auxSearch(node->right, target, depth+1, distanceTol, ids);
		}
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		auxSearch(this->root, target, 0, distanceTol, ids);
		
		return ids;
	}
	

};




