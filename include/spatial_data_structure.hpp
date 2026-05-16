#pragma once

#include <vector>
#include <memory>

#include "structs.hpp"
#include "kdTree.hpp"

class SpatialDataStructure {

public:
    SpatialDataStructure(std::vector<Point> const& points, size_t maxLeafSize = DEFAULT_MAX_LEAF_SIZE) : m_points(points), tree(m_points) {
        tree.build(maxLeafSize);
    }

    virtual ~SpatialDataStructure() = default;

    std::vector<Point> const& getPoints() const {
        return m_points;
    }

    const kdTree& getTree() const {
        return tree;
    }

    void rebuildTree(size_t maxLeafSize) {
        tree.build(maxLeafSize);
    }

private:
    std::vector<Point> m_points;
    kdTree tree;

};
