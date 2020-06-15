/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		Node** currN = &root;
		bool compY = false;
		
		while (1) {
			if (*currN == nullptr) {
				*currN = new Node(point, id);
				break;
			}else if ((*currN)->point[static_cast<int>(compY)] <= point[static_cast<int>(compY)]){
				currN = &(*currN)->left;
				compY = !compY;
			}else {
				currN = &(*currN)->right;
				compY = !compY;
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




