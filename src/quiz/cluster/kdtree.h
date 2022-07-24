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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			int current_depth = depth % 3;

			if((point[current_depth]) < ((*node)->point[current_depth]))
				insertHelper(&((*node)->left),depth+1, point, id);
			else
				insertHelper(&((*node)->right),depth+1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root
		insertHelper(&root,0,point,id);

	}

	//Helper function for Kd-tree search, recursive calling
	void serachHelper(std::vector<float> target,Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{
			bool points_valid;
			for (size_t i = 0; i < target.size(); i++) 
			{
				points_valid &= (std::abs(node->point[i] - target[i]) <= distanceTol);
			}
			
			if(points_valid)
			{
				float distance;
				for (size_t i = 0; i < 3; i++) 
				{
        			distance += (node->point[i] - target[i]) * (node->point[i] - target[i]);
      			}
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			//check across boundary with 3d points
			if((target[depth%3]-distanceTol)<node->point[depth%3])
				serachHelper(target,node->left,depth+1,distanceTol,ids);
			if((target[depth%3]+distanceTol)>node->point[depth%3])
				serachHelper(target,node->right,depth+1,distanceTol,ids);
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		serachHelper(target,root, 0, distanceTol, ids);
		return ids;
	}
	

};




