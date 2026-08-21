# Exact and Efficient Mesh-Kernel Generation

*Computer Graphics Project, Summer Semester 2026, Technische Universität Berlin*

*Joel Steinhardt*

*Based on the work of: [J. Nehring-Wirxel, P. Kern, P. Trettner, L. Kobbelt, "Exact and Efficient Mesh-Kernel Generation for Robust Geometric Computations", Computer Graphics Forum, 2025](https://www.graphics.rwth-aachen.de/publication/03355/)*

This project contains an implementation of the algorithm proposed by Nehring-Wirxel et al. for computing the kernel of a 3D mesh in an exact and efficient manner. The project extends on the results of the original paper by implementing three different parallelization strategies to accelerate the kernel computation and comparing their performance.


## 1. Running the Code

Assuming Ubuntu/Debian

```bash
sudo apt install build-essential
sudo apt install xorg-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev # Polyscope dependencies
```

In the root directory, run:

```bash
cmake . -B build -DCMAKE_BUILD_TYPE=RelWithDbInfo # BUILD_TYPE can also be `Release` or `Debug`
cmake --build build --parallel
```

To run the application, then run:

```bash
./build/bin/project
```

In the application, you can select one of the available meshes from the drop-down menu in the top right corner. More meshes in OFF format can be added to the `off_files` folder. To generate the kernel, click either "Generate Kernel" or "Generate Kernel Parallel". To step through the generation algorithm, check "Update Visuals During Stepping" and click on "Start Kernel Stepping". Now you can step through the algorithm using "Next Step", or finish the generation immediately using "Finish Kernel". 


## 2. Definition

The kernel $\cal K$ of a mesh $M$ is the set of points $\bold p$ within $M$ from which every point on the surface of $M$ is visible. It can be defined as the intersection of negative half-spaces of all supporting planes $p_i$ of the mesh's faces.

$$\mathcal{K}(M) = \{\bold p \in \mathbb{R}^3 : \forall i : n^\top_i \bold p + d_i \leq 0\}$$

Here, $n_i$ are outward pointing normals and $d_i$ are the offsets of the supporting planes. This implicitely defines a supporting plane as $p_i = n^\top_i \bold p + d_i = 0$ for face $f_i$.

Use cases for computing the kernel include autonomous exploration or star-shaped decomposition of meshes.

## 3. Algorithm Implementation

### 3.1 Technology

The implementation of the mesh-kernel generation algorithm is written in C++. The project integrates several libraries and tools:

- **Polygon Mesh Processing (PMP) Library:** Utilized as the foundation data structure for handling 3D half-edge surface meshes and topological neighborhood queries.
- **Integer Plane Geometry (IPG):** From the authors of the original paper, this library provides the exact integer arithmetic framework and robust plane-based predicates required for the intersection calculations and half-space vertex classifications.
- **Polyscope:** Employed for the visual application, the 3D visualization and the graphical user interface.
- **OpenMP:** Integrated to accelerate the algorithm's performance by parallelizing the evaluation and processing of independent cutting plane groups.

Alongside the main application, a Python script is provided for the procedural generation of test meshes in different resolutions.

### 3.1 The Base Algorithm

The base algorithm for computing the kernel works in an iterative manner: It iterates over all supporting planes of $M$ and performs polygon-plane cuts (see Section 3.1.2) on an intermediate kernel $\hat{\cal K}$ until either all planes have been processed or $\hat{\cal K}$ becomes empty. $\hat{\cal K}$ is initialized as the axis-aligned bounding box (AABB) of $M$. 

The image below illustrates the algorithm in 2D. The blue polygon represents the intermediate kernel $\hat{\cal K}$ and the red lines represent the supporting lines of the mesh. The arrows indicate the normals and lie in the positive half-space of the supporting lines.

![Algorithm Explanation 2D](img/algorithm_explanation.png)

#### 3.1.1 Exact Integer Arithmetic Foundation

To ensure an exact computation of the kernel, this project relies on the exact integer arithmetic framework Integer-Plane Geometry (IPG) introduced by Nehring-Wirxel et al.. This framework was not authored as part of this project, but is rather integrated as a foundational dependency.

Standard floating-point arithmetic is inherently susceptible to numerical inaccuracies, which can cause topological inconsistencies, incorrect outputs, or crashes during mesh intersection operations. The IPG framework circumvents these issues by adopting a purely plane-based representation. In this system, all geometric primitives are defined by one or more hyperplanes represented by integer coefficients. For example, instead of relying on floating point spatial coordinates, a point is represented as the intersection of three non-coplanar hyperplanes.

Before the kernel computation begins, the bounding box of the input mesh (from which the intermediate kernel is later instantiated) is scaled and its coordinates are rounded to a 26-bit integer grid. This ensures that all subsequent operations–such as calculating plane intersections and classifying whether points lie in front of or behind a cutting plane–can be computed using up to 256-bit exact integer arithmetic without encountering any rounding errors. Also, with this framwork, degenerate kernels can be detected and handled correctly. Note, that this system is only fully exact after the initial scaling of the input mesh, but represents "most floating point inputs exactly" as Nehring-Wirxel et al. state.

#### 3.1.2 Polygon Plane Cutting

The polygon-plane cutting operation is the core geometric operation of the algorithm. Given a convex mesh $M$ (which, in the case of the kernel computation, will be the intermediate kernel $\hat{\cal K}$) and a cutting plane $\bold p$, the goal of this algorithm is to cut $M$ at $\bold p$, discard the portion of $M$ lying in the positive half-space, and fill the resulting hole with a new cap face. This operation is based on the work of Nehring-Wirxel et al. and is implemented utilizing the exact integer arithmetic framework of IPG.

##### (Multi-Start) Edge Descent

The algorithm first aims to find a single starting edge of $M$ that intersects with $\bold p$. To achieve this efficiently, we start at an arbitrary vertex and perform a greedy edge descent, continuously moving to an adjacent vertex that minimizes the absolute distance to $\bold p$. During each step, we evaluate if the traversed half-edge crosses $\bold p$. If a crossing is detected, edge descent terminates, and this half-edge is returned as the starting point for the Marching step.

Because edge descent may get stuck in a local minimum without finding a crossing, a multi-start strategy is employed. If the initial descent from the arbitrary vertex fails, the process is re-seeded using the six extreme vertices of the mesh's AABB. Empirical testing shows that it is unlikely though not impossible for the multi-start edge descent to fail when a valid crossing edge actually exists. To guarantee robustness, if all seeded attempts fail, the algorithm falls back to a linear search over all edges.

If the linear search also yields no crossing edges, it is concluded that $\bold p$ does not intersect $M$ at all (note, that many of these non-intersect cases are already filtered out by the bounding box checks, see Section 3.2.1). At this point, a single vertex is evaluated. If it lies in the positive half-space, the enitre mesh is discarded as the kernel is empty and if it lies in the negative half-space, the cut is skipped as it does not remove any points from the intermediate kernel.

##### Marching and Filling

Once an initial crossing half-edge is identified, a localized search is performed. Starting from the vertex of the crossing half-edge that lies strictly in the positive half-space, a Depth-First Search (DFS) traverses the half-edges of the mesh.

During this traversal, the IPG framework is used to classify only the visited neighbors. The DFS exclusively explores the positive half-space; traversal along a path stops as soon as a vertex on the plane (class 0) or in the negative half-space (class < 0) is encountered. Edges connecting a positive vertex to a negative vertex are recorded as the crossing edges that will form the cut boundary.

The exact intersection point for each crossing edge is then computed using IPG. Because vertices not reached by the DFS are guaranteed to lie in the negative half-psace, they bypass the expensive exact classification and are directly mapped to the new, clipped mesh. Faces split by the cutting plane are subsequently rebuild utillizing the newly computed exact intersection vertices.

Because $\hat{\cal K}$ is strictly convex, the newly generated intersection vertices form a single, closed planar polygon. The boundary vertices are gathered and radially sorted around their centroid. This sorting ensures that the new cap face will not be self-intersecting. Finally, $\bold p$ is assigned as the exact supporting plane property of the new cap face.


### 3.2 Non-parallel Optimizations

The main operation of this algorithm is the polygon-plane cut, which can be expensive. There are several optimizations that Nehring-Wirxel et al. propose to accelerate the algorithm that were implemented in this project.

#### 3.2.1 Bounding Box Checks

With larger meshes, it is likely that a cut, specifically a cut at a convex face, misses $\hat{\cal K}$ entirely and will not have any effect. To avoid performing expensive polygon-plane intersection tests, we maintain an AABB of $\hat{\cal K}$ by storing its minimum and the maximum vertex, `aabb_min` and `aabb_max`.

Now, before performing a cut, we classify this AABB against the cutting plane.

- If the cutting plane intersects the AABB, we perform the cut as we would have done before.
- If the cutting plane lies fully in the negative half-space, we can skip the cut as nothing would be removed from $\hat{\cal K}$.
- If the cutting plane lies fully in the positive half-space, we can terminate and return an empty kernel as this cut would remove all points from $\hat{\cal K}$.

When a cut is performed on $\hat{\cal K}$, we update the AABB accordingly by finding the new minimum and maximum vertex. This avoids recomputing the AABB from scratch after every cut.

#### 3.2.2 Convex/Concave Classification

Before performing cuts, we can classify the faces of the mesh as either convex or concave by computing the signed volume between two adjacent faces $f_0$ and $f_1$. For this, we iterate over all edges $e_i$ in $M$ and find the two incident faces $f_0$ and $f_1$ to $e$. We find a vertex $v$ that does not share the edge $e$ and classify it against the plane of $f_0$ using the `classify()` function of the IPG framework. If this classification returns a positive value, $v$ sits above the plane of $f_0$ and both $f_0$ and $f_1$ are concave. In result, we store a boolean vector indicating the convexity of each face.

Using this information, we can perform some further optimizations.

If all faces are convex, we can skip the kernel computation entirely and return the mesh itself as for a fully convex mesh, the kernel is equal to the mesh. We also use the Euler-Poincaré formula to check if the mesh has a genus of greater than zero: $\chi(M) = v-e+f = 2(1-g)$ with $v$ being the number of vertices, $e$ the number of edges $f$ the number of faces, and $g$ the genus of the mesh. We check $\chi(M) < 2$. If this holds true, it implies $g > 0$ and the mesh kernel is guaranteed to be empty. Then, we can return early.

We also sort the supporting planes based on the convexity of their corresponding faces. We want to prioritize concave cuts as they are more likely to reduce the kernel volume early on. Also, this will shrink the AABB of $\hat{\cal K}$ early on, making cut skips due to missing the AABB more likely and reducing the overall amount of cuts that need to be performed.

#### 3.2.3 Coplanar Grouping

If two faces are coplanar, their supporting planes are identical and performing cuts with both planes would be redundant. Thus, we group coplanar faces together and only perform one cut. We iterate over all non-boundary edges, find their incident faces, and check if their supporting planes are parallel and if their plane coefficients match. If both conditions hold, the two faces are considered coplanar and we merge them using a `unite()` operation in a Union-Find data structure. Later code then pushes only one representative of each coplanar group to the list of supporting planes. Note that this technique will not find disconnected coplanar regions.

### 3.3 Parallelization

To further optimize the algorithm, we can parallelize the cut operations as these constitute the most computationally expensive phase of the algorithm. The core concept is to partition the supporting planes into independent batches and perform the cuts within each batch concurrently. This results in one local intermediate kernel per batch. However, rather than computing a geometric boolean intersection of these local kernels – which would be computationally heavy – the parallel phase is utilized as a filtering step. The final kernel is then generated by harvesting only the planes that survive this local culling and apply them in a final sequential pass.

For this project, three grouping strategies were implemented and tested. Each strategy creates eight batches:

- **Spatial Octants:** For each face, we compute the centroid and determine the correct octant relative to the center of the mesh's AABB. Each octant is a batch.
- **Similar Normals:** For each face, we consider its normal. All faces for which the normal points to the same directional octant are grouped together.
- **Dissimilar Normals:** For each face, we consider its normal. Planes of similar normal direction (see previous strategy) are distributed to the eight batches in a round-robin fashion.

The parallelization is implemented using OpenMP. Each batch is processed in parallel, with each thread starting with the full AABB of $M$ and trimming it using only its assigned subset of planes. If the local intermediate kernel of a thread becomes empty, the thread flags the global kernel as empty. After all batches have been processed, the algorithm iterates over the faces of the non-empty local intermediate kernels to extract their exact planes. Any plane that was cut away during concurrent processing is permanently discarded. These surviving planes are merged into a master list and applied to a new bounding box in a final sequential pass to obtain the final kernel.

## 4. Results

The algorithm was evaluated against a diverse set of 3D meshes to analyze its performance, scalability, and robustness. The test dataset comprises real-world meshes sourced from the [Thingi10k dataset](https://ten-thousand-models.appspot.com/) and procedurally generated meshes created using the provided Python script. Some of these meshes are provided in the `off_files` folder. The procedural meshes–designated as `hourglass_x`, `wheel_x`, `gear_x`, and `wad_x`, where `x` denotes the subdivision resolution–were designed to isloate specific topological traits such as heavy concavity, coplanarity, and varying bounding-box intersections. 

| `hourglass_x` | `x=4` | `x=8` | `x=16` | `x=32` | `x=64` | `x=128` |
|---------------|-------|-------|--------|--------|--------|---------|
| # vertices    | 22    | 74    | 274    | 1058   | 4162   | 16514   |
| # faces       | 40    | 144   | 544    | 2112   | 8320   | 33024   |

| `wheel_x`     | `x=4` | `x=8` | `x=16` | `x=32` | `x=64` | `x=128` |
|---------------|-------|-------|--------|--------|--------|---------|
| # vertices    | 38    | 138   | 530    | 2082   | 8258   | 32898   |
| # faces       | 72    | 272   | 1056   | 4160   | 16512  | 65792   |

| `gear_x`      | `x=4` | `x=8` | `x=16` | `x=32` | `x=64` | `x=128` |
|---------------|-------|-------|--------|--------|--------|---------|
| # vertices    | 42    | 146   | 546    | 2114   | 8322   | 33026   |
| # faces       | 80    | 288   | 1088   | 4224   | 16640  | 66048   |

| `wad_x`       | `x=4` | `x=8` | `x=16` | `x=32` | `x=64` | `x=128` |
|---------------|-------|-------|--------|--------|--------|---------|
| # vertices    | 26    | 114   | 482    | 1986   | 8066   | 32514   |
| # faces       | 48    | 224   | 960    | 3968   | 16128  | 65024   |

### 4.1 Sequential Performance

A primary observation from the benchmarking is that the total face count of a mesh is not the sole, nor even the primary, determinant of the algorithm's runtime. The generation runtime is heavily dictated by the mesh's topology, the nature of its triangulation, and its overall convexity. Two meshes with identical face counts can have drastically different runtimes depending on the frequency of actual plane-polyhedron intersections versus the number of cuts successfully skipped due to bounding box checks.

This discrepancy is visible when comparing real-world meshes. For example, the `vase` model (1,670 faces) computes in approximately 0.076 on average, while the `dreidel` model (380 faces) takes around 0.192 seconds. Despite the `vase` having more than four times the number of geometry, it evaluates considerably faster.

This topology-driven baseline is further highlighted by the procedurally generated stress tests. The `hourglass_128` mesh (approx. 33k faces) requires 98.7 seconds to compute sequentially, triggering only 2 AABB skips. Conversely, the `wad_128` mesh (approx. 65k faces) completes in only 6.8 seconds but skips more than 38k cuts due to the many shallow omni-directional dimples in its geometry. The key to high performance lies in rapidly reducing the intermediate kernel volumne to maximize AABB culling.

Also, certain geometric profiles are significantly faster to compute than others. For instance, the `gear_x` shape, which is essentially a 2D profile extruded along the z-axis, computes its kernel rapidly due the majority of faces sharing parallel or identical supporting planes. The algorithms union-find clustering step aggregates these coplanar faces, drastically reducing the effective number of unique cuts that must be performed.

### 4.2 Parallelization and Strategy Evaluation

The algorithms parallelization produces highly variable, yet occasionally massive speedups.

[Insert Figure here: Chart comparing sequential vs parallel runtimes for a procedural dataset]

For symmetric, deeply concave shapes, parallel execution yields massive performance gains. The `hourglass_128` mesh jumps from 79.47 seconds sequentially to just 4.33 seconds in parallel, an 18.35x speedup, and the `wheel_128` mesh improves from 96.02 seconds to 13.42 seconds, a 7.15x speedup. Since the algorithm only utilizes eight threads, these results suggest that for some meshes, parallelization does not only benefit from thread efficiency but alters the algorithmic complexity of the cut order. Some threads might, for example, drastically shrink the the bounding box early on, allowing all subsequent planes in that thread to be skipped, something which would not happen in the sequential algorithm.

However, for meshes with already low sequential overload, parallelization is less effective. The `gear_128` mesh, for instance, yields a negligible 1.19x speedup (0.12 s vs. 0.101 s), with parallel runs to underperform sequential ones, likely caused by the overhead of thread management.

The effectiveness of the parallelization is further modulated by the plane distribution strategy that were introduced in Section 3.3. While no single strategy dominates universally, structureal symmetries seem to dictate which approach is better.

 - For the `wad_x` mesh, dissimilar normals drastically outperform the alternatives.
 - For `gear_x`, similar normals are superior.
 - For asymmetric shapes like the `vase`, the spatial octants strategy seems to introduce significant load imbalances, resulting in slower execution times compared to the sequential algorithm, while dissimilar normals performs better than sequential.

 For the two strategies that group planes by normal direction, a theory is that similar normals are more effective when the mesh is untwisted while dissimilar normals are more effective for spiked shapes like stars because every single thread receives planes from all sides of the mesh, allowing every thread to independently build a local kernel that is more likely to be a good approximation of the final kernel. The spatial octants strategy is likely effective when the mesh is well distributed across the octants and the bounding box shrinks significantly during the parallel phase.

 [Insert Figure: Graph mapping parallelization strategy performance across the four procedural meshes]
 [Insert Figure: AABB skip rates categorized by parallelization strategy]


 ### Notizzettel 


- hourglass_x, wheel_x, gear_x, wad_x are the four procedurally generated meshes. The `x` indicates the resolution scale. [Attach tables]
- Overall, coming up with one single result is impossible. Generation runtime depends on the number of faces, the triangulation, the topology, and the convexity of the mesh. Two meshes with the same number of faces can have drastically different runtimes, dependend on how many cuts are performed and how many of them actually intersect with the intermediate kernel.
- Take these two for example: vase 1670 faces (~ 0.076 s), dreidel 380 faces (~ 0.192 s). Even though the vase has more the four times the number of faces, it is still computed faster than the dreidel.
- Topology dictates baseline: hourglass_128 (33k faces) takes 98.7 s -> 2 skips, wad_128 (65k faces) takes 6.8 s -> 38.259 skips
- Key is to reduce number of cuts that need to be performed.
- Another example is the procedually generated gear_x shape where the kernel is computed really fast in comparison to other shapes, because it is just a 2D shape stretched in the z-direction. Then, we have already eliminated two normal directions for sim
- Parallilization produces huge speedups for some meshes. For example, for wheel_128 (seq. 96.02 s vs. par. 13.42 s) which is a 7.15x speedup or hourglass_128 (seq. 79.47 s vs. 4.33 s) which is a 18.35x speedup. However, for other meshes, the parallelization does not produce a significant speedup. For example, for gear_128 (seq. 0.12 s, par. 0.101 s) which is only a 1.19x speedup on average with some parallel runs being even slower than some sequential runs. This is likely due to the fact that the parallelization overhead is not worth it for meshes where a lot of cuts are skipped and the sequential algorithm is already quite fast.
- Observed that for some meshes, symmetry seems to be a good indicator for parallelization speedup. For example, wheel_x and hourglass_x are symmetric and produce large speedups, while for vase, which is asymmetric, the parallelization does not produce a significant speedup (here, the spatial octants strategy is actually a lot slower than the sequential algorithm whereas dissimilar normals is faster).
- Parallelization strategies: No clear winner, for most meshes all three strategies perform similarly. There are however meshes where one strategy is better than the others. For example, for wad_x, the dissimilar normals strategy is superior [diagram here] (largely due to how many cuts where skipped [diagram here]) while for gear_x, the similar normals strategy is superior [diagram here].
- Theories: Similar Normals is better than other strategies if the bbox does not shrink much during parallel phase. For untwisted shapes with many similar normals. Spatial Octants is good if ?. Dissimilar Normals is good for twisted shapes like stars or other shapes with spikes.

- wheel_x: Very brutal because many cutting planes missing the mesh but still intersect with the AABB.

# Notes

- We iterate over all faces and classify them as either convex or concave.

- Supporting planes are sorted based on the convexity of their corresponding faces. Concave faces are prioritized as they are more likely to reduce the kernel volume early on.

- We maintain a bounding box for $\hat{\cal K}$ and update it after each cut.

- We iterate over the list of supporting planes and perform cuts on $\hat{\cal K}$ using the planes. For this, we check if the plane intersects the bounding box of $\hat{\cal K}$. If it does, we perform a cut and update the bounding box.

- If the bounding box of $\hat{\cal K}$ lies fully in the negative half-space of the plane, we can skip the cut as it does not cut anything away from our intermediate kernel.
- If the bounding box of $\hat{\cal K}$ lies fully in the positive half-space of the plane, we can terminate early and return an empty kernel as this cut would remove all points from our intermediate kernel.