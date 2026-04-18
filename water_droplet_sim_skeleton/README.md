# Real-Time Water Droplet Simulator Skeleton

This is a C++ starter architecture for a CS184 final project: a real-time, interactive water droplet simulator using an explicit deformable surface mesh.

The planned project scope is:

- Single-droplet deformable surface simulation core.
- Real-time refractive shading from an explicit droplet mesh.
- Dragging interaction through a local force field.
- Adjustable friction, viscosity, surface tension, contact-angle, and volume-correction parameters.
- Flat plane and vertical glass presets through the same `PlaneSurface` abstraction.
- Extensions: multiple droplets, simple merge, height-field support surface, and simple split.
- Experiment logging for performance, volume preservation, and parameter studies.

This repository is intentionally a skeleton. The CPU-side architecture and baseline logic are included. The OpenGL renderer and ImGui UI are provided as clear stubs so the team can fill in the graphics code without mixing rendering details into the simulation core.

## Suggested libraries

Minimum stack:

- C++20
- CMake
- Eigen
- OpenGL / GLFW / glad
- Dear ImGui
- Optional: libigl for cotangent Laplacian, mass matrix, and normals
- Optional: stb_image for height maps and background/environment textures

The current `CMakeLists.txt` only requires Eigen so the CPU simulation core can be built before the renderer is fully implemented.

## Build

```bash
mkdir build
cd build
cmake ..
cmake --build .
./wd_skeleton
```

## First implementation pass

1. Make `PlaneSurface`, `DropletTemplate`, `DropletFactory`, and `SingleDropletSolver` work with one droplet.
2. Render the droplet mesh without refraction.
3. Add scene color/depth pass and refractive composite shader.
4. Add `DragForceField` and parameter sliders.
5. Add experiment logging.
6. Add simple merge.
7. Add height-field surface.
8. Add simple split only after the baseline is stable.
