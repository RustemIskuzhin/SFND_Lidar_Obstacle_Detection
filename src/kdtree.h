/* \author Aaron Brown */
#include <vector>
#include <cmath>


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
    int dim;
  
	KdTree()
	: root(NULL)
	{}
    void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
    {
      if (*node==NULL)
        *node = new Node(point, id);
      else
      {  
        uint cd = depth % dim;
      	if (point[cd] < ((*node)->point[cd]))
        	insertHelper(&((*node)->left), depth+1, point, id);
      	else
        	insertHelper(&((*node)->right), depth+1, point, id);
      }               
    }
                     
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
    insertHelper(&root, 0, point, id);
	}

    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if(node==NULL)
            return;
      
        bool withindistance = true;
		for(int d = 0; d < dim; d++){
			if(node->point[d] < (target[d] - distanceTol) || node->point[d] > (target[d] + distanceTol)){
				withindistance = false;
				break;
			}
		}
        if(withindistance){
          float distance = 0;
          for (int d = 0; d < dim; d++){
				distance += (node->point[d] - target[d]) * (node->point[d] - target[d]);
		  }
          distance = sqrt(distance);
          if (distance <= distanceTol)
            ids.push_back(node->id);
        }
        
        if((target[depth%dim] - distanceTol)<node->point[depth%dim])
          searchHelper(target, node->left, depth+1, distanceTol, ids);
        if((target[depth%dim] + distanceTol)>node->point[depth%dim])
          searchHelper(target, node->right, depth+1, distanceTol, ids);
    }
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




