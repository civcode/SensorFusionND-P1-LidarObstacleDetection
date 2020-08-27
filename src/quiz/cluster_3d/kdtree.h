/* \author Aaron Brown */
// Quiz on implementing kd tree

#define K_DIM 3

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree() : root(NULL) {}

	void insert_non_recursive(std::vector<float> point, int id)
	{
		// Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		if (root == nullptr)
		{
			std::cout << "root is nullptr" << std::endl;
			root = new Node(point, id);
		}
		else
		{
			int depth = 0;
			Node *n = root;
			Node *parent;
			enum Side
			{
				left,
				right
			};
			Side s;
			while (n != nullptr)
			{
				std::cout << "depth: " << depth << std::endl;
				parent = n;
				if (depth++ % 2 == 0)
				{
					// split on x axis
					if (point[0] < n->point[0])
					{
						std::cout << "on x, n = n->left" << std::endl;
						n = n->left;
						s = left;
					}
					else
					{
						std::cout << "on x, n = n->right" << std::endl;
						n = n->right;
						s = right;
					}
				}
				else
				{
					// split on y axis
					if (point[1] < n->point[1])
					{
						std::cout << "on y, n = n->left" << std::endl;
						n = n->left;
						s = left;
					}
					else
					{
						std::cout << "on y, n = n->right" << std::endl;
						n = n->right;
						s = right;
					}
				}
			}
			if (s == left)
			{
				std::cout << "creating new node left\n";
				parent->left = new Node(point, id);
			}
			else
			{
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

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id)
	{

		if (*node == nullptr)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint cd = depth % K_DIM;
			if (point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left), depth++, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth++, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search_non_recursive(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		// define box
		float box_x_min = target[0] - distanceTol;
		float box_x_max = target[0] + distanceTol;
		float box_y_min = target[1] - distanceTol;
		float box_y_max = target[1] + distanceTol;

		// std::cout << "target = [" << target[0] << ", " << target[1] << "]" << std::endl;
		// std::cout << "distanceTol = " << distanceTol << std::endl;
		// std::cout << "box corrds: [" << box_x_min << ", " << box_x_max << ", " << box_y_min << ", " << box_y_max << "]\n";

		// check if node is within box
		//Node *n = root;
		//std::vector<Node*> selected_nodes;
		std::list<std::pair<Node*, uint> > explore_list {{root, 0}}; // Node and depth

		while (explore_list.size() != 0) {
			
			Node *n = (explore_list.begin())->first;
			uint depth = (explore_list.begin())->second;

			explore_list.pop_front();
			float p_x = n->point[0];
			float p_y = n->point[1];
			//std::cout << "node = [" << p_x << ", " << p_y << "]" << std::endl;

			// auto isInBox = [](std::vector<float>target, float distanceTol, Node *node) -> bool {
			// 	float box_x_min = target[0] - distanceTol;
			// 	float box_x_max = target[0] + distanceTol;
			// 	float box_y_min = target[1] - distanceTol;
			// 	float box_y_max = target[1] + distanceTol;

			// 	float p_x = node->point[0];
			// 	float p_y = node->point[1];

			// 	return (box_x_min <= p_x && box_x_max >= p_x && box_y_min <= p_y && box_y_max >= p_y);
	
			// };

			//isInBox(target, distanceTol, n)
			if (box_x_min <= p_x &&
				box_x_max >= p_x &&
				box_y_min <= p_y &&
				box_y_max >= p_y) {

				// calculate distance
				float d = sqrt(pow(target[0] - p_x, 2) + pow(target[1] - p_y, 2));
				//std::cout << "d = " << d << std::endl;
				if (d <= distanceTol)
				{
					//std::cout << "add id " << n->id << std::endl;
					ids.push_back(n->id);
					//selected_nodes.push_back(n);
				}
			}
			// check if children should be visited
			if (depth%2 == 0) {
				// x-axis split
				if (box_x_min <= p_x && n->left != nullptr) {
					explore_list.push_back({n->left, depth});
				} 
				if (box_x_max >= p_x && n->right != nullptr) {
					explore_list.push_back({n->right, depth});
				}
			} else {
				// y-axis split
				if (box_y_min <= p_y && n->left != nullptr) {
					explore_list.push_back({n->left, depth});
				} 
				if (box_y_max >= p_y && n->right != nullptr) {
					explore_list.push_back({n->right, depth});
				}
			}
		}

		return ids;
	}

	void searchHelper(std::vector<float>target, Node* node, uint depth, float distanceTol, std::vector<int>& ids) {

		auto isInBox = [](std::vector<float>target, float distanceTol, Node *node) -> bool {
			float box_x_min = target[0] - distanceTol;
			float box_x_max = target[0] + distanceTol;
			float box_y_min = target[1] - distanceTol;
			float box_y_max = target[1] + distanceTol;
			float box_z_min = target[2] - distanceTol;
			float box_z_max = target[2] + distanceTol;

			float p_x = node->point[0];
			float p_y = node->point[1];
			float p_z = node->point[2];

			return (box_x_min <= p_x && 
					box_x_max >= p_x && 
					box_y_min <= p_y && 
					box_y_max >= p_y &&
					box_z_min <= p_z && 
					box_z_max >= p_z );
		};

		if (node != nullptr) {

			if (isInBox(target, distanceTol, node)) {
				float d = sqrt(	pow(target[0] - node->point[0], 2) + 
							  	pow(target[1] - node->point[1], 2) + 
								pow(target[2] - node->point[2], 2) );
				if (d <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			if (target[depth%K_DIM]-distanceTol <= node->point[depth%K_DIM])
				searchHelper(target, node->left, depth++, distanceTol, ids);
			if (target[depth%K_DIM]+distanceTol >= node->point[depth%K_DIM])
				searchHelper(target, node->right, depth++, distanceTol, ids);
		}

	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
};
