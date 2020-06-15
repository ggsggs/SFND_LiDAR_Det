/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node *root;

  KdTree() : root(NULL) {}

  void insert(std::vector<float> point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
    Node **currN = &root;
    bool compY = false;

    while (1) {
      if (*currN == nullptr) {
        *currN = new Node(point, id);
        break;
      } else if (point[static_cast<int>(compY)] <
                 (*currN)->point[static_cast<int>(compY)]) {
        currN = &(*currN)->left;
        compY = !compY;
      } else {
        currN = &(*currN)->right;
        compY = !compY;
      }
    }
  }

  bool isPointIn(const std::vector<float> &target,
                 const std::vector<float> &point, float distanceTol) {
    bool isIn = true;
    std::vector<float> distV;
    for (size_t i = 0; i < target.size(); i++) {
      distV.push_back(fabs(target[i] - point[i]));
      if (distV[i] > distanceTol) {
        isIn = false;
        break;
      }
    }
    if (isIn) {
      float dist = 0;
      for (size_t i = 0; i < target.size(); i++)
        dist += pow(distV[i], 2);
      isIn = sqrtf(dist) < distanceTol;
    }
    return isIn;
  }

  void searchSubTrees(std::vector<float> target, float distanceTol,
                      std::vector<int> &ids, uint depth, const Node *n) {
    int idx = depth % target.size();
    if (n == nullptr)
      return;
    bool isIn = isPointIn(target, n->point, distanceTol);
    if (isIn) {
      ids.push_back(n->id);
      searchSubTrees(target, distanceTol, ids, depth + 1, n->left);
      searchSubTrees(target, distanceTol, ids, depth + 1, n->right);
    } else {
      searchSubTrees(target, distanceTol, ids, depth + 1,
                     target[idx] < n->point[idx] ? n->left : n->right);
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    Node *n = root;
    searchSubTrees(target, distanceTol, ids, 0, n);
    return ids;
  }
};
