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

To run the application, then run:

```bash
./build/bin/project
```