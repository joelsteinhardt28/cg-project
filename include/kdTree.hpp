#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include <numeric>
#include <algorithm>
#include <queue>

#include "structs.hpp"


// * Node structure for the KD-tree
struct KDNode {
    Point p;        // Splitting point (if internal node)
    size_t idx;     // Original index from the input dataset
    bool isLeaf;
    std::vector<size_t> pointIndices;  // Indices of points in the leaf node (only for leaf nodes)
    
    // Pointers to the child nodes in the KD-tree
    KDNode* left;
    KDNode* right;

    // The two corners of the bounding box for this node's subtree
    Point minBound;  
    Point maxBound;
    
    // Constructors for internal nodes and leaf nodes
    KDNode(Point const& point, size_t index) : p(point), idx(index), isLeaf(false), left(nullptr), right(nullptr) {}
    KDNode(std::vector<size_t> const& indices) : isLeaf(true), pointIndices(indices), left(nullptr), right(nullptr) {}
};


// KD-Tree for 3D points
class kdTree {

public:
    // Constructor and destructor
    kdTree(std::vector<Point> const& points);
    ~kdTree();

    KDNode* getRoot() const { return root; };
    void getSplittingPlanes(KDNode* node, int depth, std::vector<Point>& vertices, std::vector<Face>& faces) const;

    // Prevent copying to avoid double-free mem corruption on root pointer
    kdTree(const kdTree&) = delete;
    kdTree& operator=(const kdTree&) = delete;

    void build(size_t maxLeafSize = DEFAULT_MAX_LEAF_SIZE);
    void print() const;

    // Queries
    std::vector<std::size_t> collectInRadius(Point const& p, float radius) const;
    std::vector<std::size_t> collectKNearest(Point const& p, unsigned int k) const;

private:
    const std::vector<Point>& points;  // Reference the points, no moving, no copying
    KDNode* root;

    KDNode* buildRecursive(std::vector<size_t>& indices, size_t start, size_t end, int depth, Point currentMin, Point currentMax, size_t maxLeafSize);
    void clear(KDNode* node);

    void printRecursive(KDNode* node, int depth) const;

    void collectInRadiusRecursive(KDNode* node, Point const& p, float radiusSquared, int depth, std::vector<std::size_t>& result) const;
    void collectKNearestRecursive(KDNode* node, Point const& p, unsigned int k, int depth, std::priority_queue<std::pair<float, size_t>>& knnQueue) const;

};