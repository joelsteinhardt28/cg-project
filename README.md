# Computer Graphics Project

Computer Graphics Project at TU Berlin, SS26. Topic: Exact and Efficient Mesh-Kernel Generation.

## Requirements and Compilation

Assume Ubuntu/Debian:

```bash
sudo apt install build-essential
sudo apt install xorg-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev # Polyscope dependencies
```

In the root directory, run:

```bash
cmake . -B build -DCMAKE_BUILD_TYPE=RelWithDbInfo # BUILD_TYPE can also be `Release` or `Debug`
cmake --build build --parallel
```

To run the application:

```bash
./build/bin/project
```

## How to Use the UI

1. **Mesh Loading & Reset**:
   - Select an `.off` mesh file from the **OFF File** dropdown list under the **Mesh Loading** section.
   - Additional `.off` files placed in the `./off_files` directory are automatically scanned and listed.
   - Click the **Reset** button at the bottom of the loading section to reload the active mesh and reset all computed kernel/visual state.

2. **Mesh Kernel Generation**:
   - **Generate Kernel**: Computes the 3D mesh kernel using the sequential plane-cutting algorithm.
   - **Generate Kernel Parallel**: Computes the kernel in parallel using multi-threaded plane group slicing via OpenMP.
   - **Show / Hide Cut Plane & Normal**: Toggles the visibility of the active cutting plane quad and its normal vector in the 3D viewport.
   - **In-App Feedback & Warnings**: Early termination and empty kernel notifications (e.g., genus $> 0$, fully convex meshes, or empty kernels) are displayed directly in the control panel text as well as via Polyscope viewport overlay windows.

3. **Kernel Stepping**:
   - Toggle **Update Visuals During Stepping** and click **Start Kernel Stepping**.
   - Step through individual plane cuts sequentially using **Next Step**, finish all remaining cuts immediately with **Finish Kernel**, or abort with **Cancel Stepping**.

4. **Mesh Analysis & Visualization Utilities**:
   - **Visualize Face Normals**: Renders face normal vectors on the loaded surface mesh.
   - **Identify Concave Faces**: Identifies and visualizes concave faces on the surface mesh via a scalar quantity overlay.