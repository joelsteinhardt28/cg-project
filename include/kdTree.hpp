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