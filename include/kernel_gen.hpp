#pragma once

#include <pmp/bounding_box.h>
#include <pmp/algorithms/triangulation.h>
#include <pmp/algorithms/utilities.h>

#include "mesh_utils.hpp"

/**
 * Union-Find (Disjoint-Set) data structure with path compression and union by rank.
 * Used for clustering coplanar adjacent faces into connected plane groups.
 */
struct UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;

    UnionFind(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }

    // Looks up which group face i belongs to, with path compression
    int find(int i) {
        if (parent[i] == i) return i;
        return parent[i] = find(parent[i]);
    }

    // Merges two face groups together, with union by rank
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

/**
 * Represents a cluster of coplanar faces sharing a single supporting plane.
 */
struct ClusteredPlane {
    pmp::Face face;        ///< Representative root face for the cluster
    Plane plane;           ///< Floating-point support plane for visualization
    ExactPlane exactPlane; ///< Exact 256-bit representation of the support plane
    bool isConcave;        ///< True if any face within the cluster is concave
};


/**
 * Constructs an initial axis-aligned bounding box (AABB) mesh for the intermediate kernel,
 * initializing exact 26-bit integer vertex coordinates and exact 256-bit face supporting planes.
 * 
 * @param bbox The 3D bounding box bounds of the input mesh
 * @return pmp::SurfaceMesh The instantiated quad mesh representing the AABB
 */
pmp::SurfaceMesh construct_aabb_mesh(pmp::BoundingBox& bbox);

/**
 * Precomputes face supporting planes, groups connected coplanar faces using Union-Find,
 * aggregates concavity flags across each cluster, and returns unique root face plane clusters.
 * 
 * @param state The global application state
 * @param isConcaveFace Vector indicating concavity of each face in the input mesh
 * @return std::vector<ClusteredPlane> The list of unique clustered supporting planes
 */
std::vector<ClusteredPlane> extract_and_cluster_planes(AppState& state, const std::vector<bool>& isConcaveFace);

/**
 * Initializes the sequential kernel stepping process. Evaluates genus and concavity early-termination checks,
 * initializes the AABB intermediate kernel, pre-computes plane clusters, and registers visual structures.
 * 
 * @param state The global application state
 */
void init_kernel_stepping(AppState& state);

/**
 * Performs a single step in the sequential kernel generation process by cutting the intermediate kernel
 * at the next supporting plane in queue.
 * 
 * @param state The global application state
 * @param updateVisuals Whether to update Polyscope 3D visualization after the cut
 */
void step_kernel(AppState& state, bool updateVisuals);

/**
 * Executes the complete sequential mesh kernel generation algorithm from start to finish.
 * 
 * @param state The global application state
 */
void generate_kernel(AppState& state);

/**
 * Executes multi-threaded parallel kernel generation using OpenMP. Planes are partitioned into
 * 8 groups according to the selected ParallelStrategy (Spatial Octants, Similar Normals, or Dissimilar Normals),
 * cut concurrently in parallel threads, and merged for the final kernel output.
 * 
 * @param state The global application state
 */
void generate_kernel_parallel(AppState& state);