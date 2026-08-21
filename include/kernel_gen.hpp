#pragma once

#include <pmp/bounding_box.h>
#include <pmp/algorithms/triangulation.h>
#include <pmp/algorithms/utilities.h>

#include "mesh_utils.hpp"


enum class ParallelStrategy {
    SpatialOctants,
    SimilarNormals,
    DissimilarNormals
};

struct UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;

    UnionFind(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }

    // Looks up which group face i belong to, with path compression
    int find(int i) {
        if (parent[i] == i) return i;
        return parent[i] = find(parent[i]);
    }

    // Merge two groups together, with union by rank
    void unite(int i, int j) {  
        int root_i = find(i);
        int root_j = find(j);
        if (root_i != root_j) {
            if (rank[root_i] > rank[root_j]) {
                parent[root_j] = root_i;
            } else if (rank[root_i] < rank[root_j]) {
                parent[root_i] = root_j;
            } else {
                parent[root_j] = root_i;
                rank[root_i]++;
            }
        }
    }
};

struct ClusteredPlane {
    pmp::Face face;
    Plane plane;
    ExactPlane exactPlane;
    bool isConcave;
};


pmp::SurfaceMesh construct_aabb_mesh(pmp::BoundingBox& bbox);

std::vector<ClusteredPlane> extract_and_cluster_planes(AppState& state, const std::vector<bool>& isConcaveFace);

void init_kernel_stepping(AppState& state);
void step_kernel(AppState& state, bool updateVisuals);
void generate_kernel(AppState& state);
void generate_kernel_parallel(AppState& state);