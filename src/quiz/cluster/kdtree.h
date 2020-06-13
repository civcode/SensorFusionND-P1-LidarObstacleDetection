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

	KdTree() : root(NULL) {}

	void insert_non_recoursive(std::vector<float> point, int id)
	{
		// Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (root == nullptr) {
			std::cout << "root is nullptr" << std::endl;
			root = new Node(point, id);

		} else {
			int depth = 0;
			Node *n = root;
			Node *parent;
			enum Side {left, right};
			Side s;
			while (n != nullptr) {
				std::cout << "depth: " << depth << std::endl;
				parent = n;
				if (depth++%2 == 0) {
					// split on x axis
					if (point[0] < n->point[0]) {
						std::cout << "on x, n = n->left" << std::endl;
						n = n->left;
						s = left;
					} else {
						std::cout << "on x, n = n->right" << std::endl;
						n = n->right;
						s = right;
					}
				} else {
					// split on y axis
					if (point[1] < n->point[1]) {
						std::cout << "on y, n = n->left" << std::endl;
						n = n->left;
						s = left;
					} else {
						std::cout << "on y, n = n->right" << std::endl;
						n = n->right;
						s = right;
					}
				}
			}
			if (s == left) {
				std::cout << "creating new node left\n";
				parent->left = new Node(point, id);
			} else {
				std::cout << "creating new node right\n";
				parent->right = new Node(point, id);
			}
			

			// if (root->left == nullptr) {
			// 	std::cout << "root->left is nullptr" << std::endl;
			// } else if (root->right == nullptr) {
			// 	std::cout << "root->right is nullptr" << std::endl;
			// }
		}

	}

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id) {

		if (*node == nullptr) {
			*node = new Node(point, id);
		} else {
			uint cd = depth % 2;
			if (point[cd] < ((*node)->point[cd])) {
				insertHelper(&((*node)->left), depth++, point, id);
			} else {
				insertHelper(&((*node)->right), depth++, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




