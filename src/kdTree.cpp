#include "kdTree.hpp"

// Constructor and destructor
kdTree::kdTree(std::vector<Point> const& points) : points(points), root(nullptr) {}
kdTree::~kdTree() { clear(root); }


void kdTree::build(size_t maxLeafSize) {
    if (points.empty()) return;

    // Delete any existing tree before building a new one
    clear(root);
    root = nullptr;

    std::vector<size_t> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Calculate the bounding box for the entire point set
    Point globalMin = {
        std::numeric_limits<float>::max(), 
        std::numeric_limits<float>::max(), 
        std::numeric_limits<float>::max()
    };
    Point globalMax = {
        std::numeric_limits<float>::lowest(), 
        std::numeric_limits<float>::lowest(), 
        std::numeric_limits<float>::lowest()
    };

    for (const Point& p : points) {
        for (int i = 0; i < 3; ++i) {
            globalMin[i] = std::min(globalMin[i], p[i]);
            globalMax[i] = std::max(globalMax[i], p[i]);
        }
    }

    root = buildRecursive(indices, 0, indices.size(), 0, globalMin, globalMax, maxLeafSize);
}

KDNode* kdTree::buildRecursive(std::vector<size_t>& indices, size_t start, size_t end, int depth, Point currentMin, Point currentMax, size_t maxLeafSize) {
    // Base case: No points left to build
    if (start >= end) return nullptr;

    // Base case: Create Leaf node if we reach maxLeafSize (we're small enough to stop splitting)
    if ((end - start) <= maxLeafSize) {
        std::vector<size_t> leafPoints(indices.begin() + start, indices.begin() + end);
        KDNode* leafNode = new KDNode(leafPoints);
        leafNode->minBound = currentMin;
        leafNode->maxBound = currentMax;
        return leafNode;
    }

    int dim = depth % 3;  // Determine current axis (x=0, y=1, z=2)
    size_t mid = start + (end - start) / 2;

    // Median search, split points based on the current dimension
    // TODO: Replace this with an own implementation of quickselect?
    std::nth_element(
        indices.begin() + start, 
        indices.begin() + mid, 
        indices.begin() + end,
        [this, dim](size_t a, size_t b) {
            return points[a][dim] < points[b][dim];  // Sort along the current dimension
        }
    );

    // Index at mid is now the median for this dim
    size_t idx = indices[mid];

    // Create a new node with the median point
    KDNode* node = new KDNode(points[idx], idx);
    node->minBound = currentMin;
    node->maxBound = currentMax;

    // Update bounding box for left and right subtrees
    Point leftMax = currentMax;
    leftMax[dim] = points[idx][dim];  // Left subtree ends at current median
    Point rightMin = currentMin;
    rightMin[dim] = points[idx][dim];  // Right subtree starts at current median

    // Recursively build left and right subtrees
    node->left = buildRecursive(indices, start, mid, depth + 1, currentMin, leftMax, maxLeafSize);
    node->right = buildRecursive(indices, mid + 1, end, depth + 1, rightMin, currentMax, maxLeafSize);

    return node;
}


void kdTree::getSplittingPlanes(KDNode* node, int depth, std::vector<Point>& vertices, std::vector<Face>& faces) const {
    if (!node || node->isLeaf) return;

    int dim = depth % 3;  // Determine current axis (x=0, y=1, z=2)

    // Four corners of the splitting plane
    Point c1 = node->minBound;
    Point c2 = node->minBound;
    Point c3 = node->maxBound;
    Point c4 = node->maxBound;

    // Fix the coordinate along the splitting dimension
    c1[dim] = c2[dim] = c3[dim] = c4[dim] = node->p[dim];
    // Determine the other two dimensions to span the plane
    int dim1 = (dim + 1) % 3;
    int dim2 = (dim + 2) % 3;

    c1[dim1] = node->minBound[dim1];
    c1[dim2] = node->minBound[dim2];
    c2[dim1] = node->minBound[dim1];
    c2[dim2] = node->maxBound[dim2];
    c3[dim1] = node->maxBound[dim1];
    c3[dim2] = node->maxBound[dim2];
    c4[dim1] = node->maxBound[dim1];
    c4[dim2] = node->minBound[dim2];

    size_t baseIdx = vertices.size();
    vertices.push_back(c1);
    vertices.push_back(c2);
    vertices.push_back(c3);
    vertices.push_back(c4);

    faces.push_back({baseIdx, baseIdx + 1, baseIdx + 2, baseIdx + 3});

    getSplittingPlanes(node->left, depth + 1, vertices, faces);
    getSplittingPlanes(node->right, depth + 1, vertices, faces);
}


void kdTree::clear(KDNode* node) {
    if (node == nullptr) return;

    clear(node->left);
    clear(node->right);
    delete node;
}


void kdTree::print() const {
    printRecursive(root, 0);
}

void kdTree::printRecursive(KDNode* node, int depth) const {
    if (node == nullptr) return;

    // Print indention based on depth and the node itself
    for (int i = 0; i < depth; ++i) {
        std::cout << "  ";
    }
    std::cout << "(" << node->p[0] << ", " << node->p[1] << ", " << node->p[2] << ")" << std::endl;

    printRecursive(node->left, depth + 1);
    printRecursive(node->right, depth + 1);
}


// * Radius search: Collect indices of points within a given radius from the query point
std::vector<size_t> kdTree::collectInRadius(Point const& p, float radius) const {
    std::vector<size_t> result;
    collectInRadiusRecursive(root, p, radius * radius, 0, result);
    return result;
}

void kdTree::collectInRadiusRecursive(KDNode* node, Point const& p, float radiusSquared, int depth, std::vector<size_t>& result) const {
    if (!node) return;

    int dim = depth % 3;

    auto distanceSquared = [&](const Point& a, const Point& b) {
        float dx = a[0] - b[0];
        float dy = a[1] - b[1];
        float dz = a[2] - b[2];
        return (dx * dx) + (dy * dy) + (dz * dz);
    };

    // If it's a leaf node, check all points within the leaf against the query point
    if (node->isLeaf) {
        for (size_t idx : node->pointIndices) {
            if (distanceSquared(points[idx], p) <= radiusSquared) {
                result.push_back(idx);
            }
        }
        return;
    }

    // General check: Current node's point is within radius of the query point
    if (distanceSquared(node->p, p) <= radiusSquared) {
        result.push_back(node->idx);
    }

    // Determine which child is on the same side as the query point
    float distanceToPlane = p[dim] - node->p[dim];
    KDNode* nearChild = (distanceToPlane < 0) ? node->left : node->right;
    KDNode* farChild = (distanceToPlane < 0) ? node->right : node->left;

    // Always check the near child first
    collectInRadiusRecursive(nearChild, p, radiusSquared, depth + 1, result);

    // Chec far child if search sphere overlaps with the splitting plane
    if ((distanceToPlane * distanceToPlane) <= radiusSquared) {
        collectInRadiusRecursive(farChild, p, radiusSquared, depth + 1, result);
    }
}


// * K-nearest neighbor search: Collect indices of the k nearest points to the query point
std::vector<size_t> kdTree::collectKNearest(Point const& p, unsigned int k) const {
    std::priority_queue<std::pair<float, size_t>> knnQueue;

    collectKNearestRecursive(root, p, k, 0, knnQueue);

    std::vector<size_t> result;
    result.reserve(knnQueue.size());
    while (!knnQueue.empty()) {
        result.push_back(knnQueue.top().second);
        knnQueue.pop();
    }

    std::reverse(result.begin(), result.end());  // Because prio queue pops largest, not smallest
    return result;
}

void kdTree::collectKNearestRecursive(KDNode* node, Point const& p, unsigned int k, int depth, std::priority_queue<std::pair<float, size_t>>& knnQueue) const {
    if (!node) return;

    int dim = depth % 3;

    auto distanceSquared = [&](const Point& a, const Point& b) {
        float dx = a[0] - b[0];
        float dy = a[1] - b[1];
        float dz = a[2] - b[2];
        return (dx * dx) + (dy * dy) + (dz * dz);
    };

    // If it's a leaf node, check all points within the leaf against the query point
    if (node->isLeaf) {
        for (size_t idx : node->pointIndices) {
            float dsq = distanceSquared(points[idx], p);
            if (knnQueue.size() < k) {
                knnQueue.push({dsq, idx});
            } else if (dsq < knnQueue.top().first) {
                knnQueue.pop();
                knnQueue.push({dsq, idx});
            }
        }
        return;
    }
    
    // General check: Current node's point is a candidate for k-nearest neighbors
    float dsq = distanceSquared(node->p, p);
    if (knnQueue.size() < k) {
        knnQueue.push({dsq, node->idx});
    } else if (dsq < knnQueue.top().first) {
        knnQueue.pop();
        knnQueue.push({dsq, node->idx});
    }

    // Determine which child is on the same side as the query point
    float distanceToPlane = p[dim] - node->p[dim];
    KDNode* nearChild = (distanceToPlane < 0) ? node->left : node->right;
    KDNode* farChild = (distanceToPlane < 0) ? node->right : node->left;

    // Always check the near child first
    collectKNearestRecursive(nearChild, p, k, depth + 1, knnQueue);

    // Check the far child if the search sphere overlaps with the splitting plane
    if (knnQueue.size() < k || (distanceToPlane * distanceToPlane) <= knnQueue.top().first) {
        collectKNearestRecursive(farChild, p, k, depth + 1, knnQueue);
    }
}