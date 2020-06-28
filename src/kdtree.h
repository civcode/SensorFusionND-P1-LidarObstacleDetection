/* \author Aaron Brown */
// Quiz on implementing kd tree
#define K_DIM 3

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {

	Node *root;

	KdTree() : root(NULL) {}

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id) {

		if (*node == nullptr) {
			*node = new Node(point, id);
		} else {
			uint cd = depth % K_DIM;
			if (point[cd] < ((*node)->point[cd])) {
				insertHelper(&((*node)->left), depth+1, point, id);
			} else {
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id) {
		insertHelper(&root, 0, point, id);
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

			if ((target[depth%K_DIM]-distanceTol) <= node->point[depth%K_DIM])
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if ((target[depth%K_DIM]+distanceTol) >= node->point[depth%K_DIM])
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}

	}

	void searchHelperX(const std::vector<float>& target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
		if (nullptr != node) {
		float dx = fabs(node->point[0] - target[0]);
		float dy = fabs(node->point[1] - target[1]);

				if (dx <= distanceTol && dy <= distanceTol) {
					float distance = std::hypotf(dx, dy);
					if (distance <= distanceTol) {
						ids.push_back((node->id));
					}
				}

				// Check across boundary
				if ((target[depth % K_DIM] - distanceTol) < node->point[depth % K_DIM]) {
					searchHelperX(target, node->left, depth + 1, distanceTol, ids);
				}
				if ((target[depth % K_DIM] + distanceTol) > node->point[depth % K_DIM]) {
					searchHelperX(target, node->right, depth + 1, distanceTol, ids);
				}
			}
		}




	std::vector<int> search(std::vector<float> target, float distanceTol) 
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
};
