// Quiz on implementing kd tree 
#ifndef KDTREE_H
#define KDTREE_H
#include "render/render.h"
#include <memory>

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  std::unique_ptr<Node> left;
  std::unique_ptr<Node> right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(nullptr), right(nullptr) {}
};

struct KdTree {
  std::unique_ptr<Node> root;

  KdTree() : root(nullptr) {}

  void insert(std::vector<float> point, int id) {
    // DONE: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
   insertRecursive(root, point, id, 0); 
  } 

  void insertRecursive(std::unique_ptr<Node>& node, const std::vector<float>& point, const int id, const uint depth) {
    uint idx = depth % point.size();
    if (node == nullptr) node = std::unique_ptr<Node>(new Node{point, id});
    else {
      insertRecursive(point[idx] <= node->point[idx]? node->left : node->right,
                      point, id, depth+1);
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
      isIn = sqrtf(dist) <= distanceTol;
    }
    return isIn;
  }

  void searchSubTrees(const std::vector<float> &target, float distanceTol,
                      std::vector<int> &ids, uint depth, const Node *n) {
    int idx = depth % target.size();
    if (n == nullptr)
      return;
    bool isIn = isPointIn(target, n->point, distanceTol);
    if (isIn) ids.push_back(n->id);

    if (target[idx] - distanceTol < n->point[idx]);
      searchSubTrees(target, distanceTol, ids, depth + 1, n->left.get());
    
    if (target[idx] + distanceTol >= n->point[idx])
      searchSubTrees(target, distanceTol, ids, depth + 1, n->right.get());

  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(const std::vector<float> &target, float distanceTol) {
    std::vector<int> ids;
    searchSubTrees(target, distanceTol, ids, 0, root.get());
    return ids;
  }
};

#endif