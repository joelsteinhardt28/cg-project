#pragma once

#include <array>
#include <vector>
#include <memory>
#include <string>


// Forward declarations
namespace polyscope {
    class PointCloud;
    class SurfaceMesh;
}
class SpatialDataStructure;

using Point = std::array<float, 3>;
using Face = std::vector<size_t>;
using Normal = std::array<float, 3>;

inline constexpr int DEFAULT_MAX_LEAF_SIZE = 5;


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


// * Application state struct to hold shared data across the application
struct AppState {
    polyscope::PointCloud* pc = nullptr;
    polyscope::SurfaceMesh* sc = nullptr;
    std::unique_ptr<SpatialDataStructure> sds;
    std::vector<Point> bboxVertices;
    int maxLeafSize = DEFAULT_MAX_LEAF_SIZE;

    std::string targetDir = "./off_files";
    std::vector<std::string> offFiles;
    int selectedOffFileIdx = -1;

    int queryType = 0;   // 0 = none, 1 = radius, 2 = knn
    int queryK = 10;
    float queryRadius = 0.5f;
    int queryPointIdx = -1;

    // GUI
    bool showQueryDialog = false;
};