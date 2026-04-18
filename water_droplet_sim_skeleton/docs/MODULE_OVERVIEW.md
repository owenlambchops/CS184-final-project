# Module and File Overview

This document explains the purpose of every module and file in the starter code.

The main design rule is:

> `Scene` stores state; `SimulationSystem` modifies physical state; `Renderer` reads state only; `Interaction` changes force fields instead of directly editing mesh vertices; simple merge/split delete old droplets and respawn new droplets instead of performing mesh boolean operations.

---

## Top-Level Layout

```text
include/wd/
  app/
  core/
  surface/
  forces/
  sim/
  render/
  interaction/
  ui/
  experiments/

src/
  app/
  core/
  surface/
  forces/
  sim/
  render/
  interaction/
  ui/
  experiments/

assets/shaders/
docs/
```

---

## `CMakeLists.txt`

Defines the C++20 project, builds the `wd_core` library, and builds the minimal executable `wd_skeleton`.

It currently requires Eigen. GLFW, glad, ImGui, and libigl should be added after the team starts implementing the interactive renderer.

---

## `README.md`

Gives the project scope, external library plan, build instructions, and recommended implementation order.

---

## `docs/MODULE_OVERVIEW.md`

This file. It should be shared with team members so everyone knows where each feature should be implemented.

---

# Core Module

## `include/wd/core/types.h`

Defines common data structures shared by the entire project:

- Math aliases: `Vec2`, `Vec3`, `MatX3d`, `MatX3i`, `SparseMat`
- Geometry structs: `Ray`, `AABB`, `SurfaceSample`, `PickHit`
- Parameter structs:
  - `MaterialParams`
  - `SolverParams`
  - `RenderParams`
- Runtime stats:
  - `SimulationStats`
  - `RenderStats`

This file is safe to include almost anywhere.

## `include/wd/core/scene.h`

Defines `Scene`, the main state container. It owns:

- One support surface
- One composite force field
- A list of droplets

`Scene` should not contain simulation algorithms or rendering calls.

## `src/core/scene.cpp`

Implements simple ownership setters for `Scene`.

---

# Surface Module

## `include/wd/surface/isurface.h`

Defines the abstract support-surface interface:

- `closestSample()`
- `projectPoint()`
- `normalAt()`
- `raycast()`
- `bounds()`

The solver only talks to `ISurface`, so the same simulation code can run on a flat plane, vertical glass, or height field.

## `include/wd/surface/plane_surface.h`

Declares `PlaneSurface`, used for both horizontal flat planes and vertical glass panels.

## `src/surface/plane_surface.cpp`

Implements plane projection, normal query, tangent frame construction, and ray-plane intersection.

## `include/wd/surface/heightfield_surface.h`

Declares `HeightFieldSurface`, used for non-flat surfaces such as bumpy glass or a simplified artificial leaf surface.

## `src/surface/heightfield_surface.cpp`

Implements a basic height sampler, finite-difference normal estimation, point projection, and approximate ray marching. This is intentionally simple and should be improved later.

---

# Force Field Module

## `include/wd/forces/iforce_field.h`

Declares:

- `IForceField`
- `ConstantForceField`
- `DragForceField`
- `CompositeForceField`

Gravity, arbitrary vector fields, and mouse dragging are all represented as force fields.

## `src/forces/iforce_field.cpp`

Implements force-field sampling and summation. The drag field uses a compact local falloff around the mouse-picked point.

---

# Simulation Module

## `include/wd/sim/droplet_template.h`

Declares a reusable fixed-topology droplet mesh template.

The initial template is a spherical-cap-like triangular mesh with a known boundary loop.

## `src/sim/droplet_template.cpp`

Generates the initial spherical-cap mesh.

This is the first version of the droplet geometry. Later versions can replace it with a better triangulation or remeshing structure.

## `include/wd/sim/droplet_cache.h`

Defines `DropletDerivedData`, which stores quantities derived from current droplet state:

- Vertex normals
- Center of mass
- Average velocity
- Current volume estimate
- Footprint radius
- Elongation ratio
- Principal axis

## `include/wd/sim/droplet.h`

Declares the `Droplet` class. A droplet stores:

- Current vertex positions
- Current vertex velocities
- Mesh template reference
- Material parameters
- Target volume
- Derived data cache

A droplet must not own OpenGL buffers or UI state.

## `src/sim/droplet.cpp`

Computes vertex normals, center of mass, average velocity, footprint radius, elongation ratio, and principal axis.

## `include/wd/sim/droplet_factory.h`

Declares `DropletFactory`, the only module that should create new droplets.

It supports:

- Normal spawn
- Spawn from merge
- Spawn from split

## `src/sim/droplet_factory.cpp`

Places a local droplet template on a support surface using the surface tangent frame. It also creates merged and split droplets using conserved target volume and mass-weighted average velocity.

## `include/wd/sim/operators.h`

Declares individual simulation operators:

- `ExternalForceOperator`
- `CollisionProjector`
- `ViscosityOperator`
- `CurvatureFlowOperator`
- `ContactLineOperator`
- `VolumeCorrector`

The current operators are baseline versions. They can later be replaced by cotangent-Laplacian and contact-angle implementations.

## `src/sim/operators.cpp`

Implements the baseline physical operators:

- Project external force to the support-surface tangent plane
- Semi-contact collision projection
- Simple damping and placeholder Laplacian viscosity
- Uniform-Laplacian smoothing as a surface-tension proxy
- Boundary pinning and friction
- Approximate closed-volume computation and height-based volume correction

## `include/wd/sim/single_droplet_solver.h`

Declares `SingleDropletSolver`, which coordinates all per-droplet operators.

## `src/sim/single_droplet_solver.cpp`

Runs one simulation step:

1. External force
2. Semi-implicit integration
3. Collision projection
4. Viscosity
5. Curvature flow
6. Contact-line correction
7. Volume correction
8. Derived-data update

## `include/wd/sim/merge_split_controller.h`

Declares `MergeSplitController`, responsible for simple topology events.

## `src/sim/merge_split_controller.cpp`

Implements simple merge and split:

- Merge: if two droplets are close, delete both and respawn one larger droplet.
- Split: if a droplet is sufficiently elongated and large enough, delete it and respawn two smaller droplets.

This baseline does not perform mesh cutting or boolean union.

## `include/wd/sim/simulation_system.h`

Declares `SimulationSystem`, the scene-level simulation entry point.

## `src/sim/simulation_system.cpp`

Steps all droplets, processes merge/split events, advances simulation time, and records simulation stats.

---

# Rendering Module

## `include/wd/render/gpu_mesh_buffer.h`

Declares a GPU mesh buffer wrapper for one droplet.

## `src/render/gpu_mesh_buffer.cpp`

Stub implementation. Add VAO/VBO/EBO creation, vertex uploads, and draw calls here.

## `include/wd/render/droplet_gpu_cache.h`

Declares a cache mapping droplet IDs to GPU mesh buffers.

## `src/render/droplet_gpu_cache.cpp`

Creates, updates, and removes droplet GPU buffers based on the current droplet list.

## `include/wd/render/refractive_renderer.h`

Declares the top-level renderer and a minimal `Camera` placeholder.

Planned passes:

1. Scene color/depth pass
2. Droplet normal/depth/thickness pass
3. Refractive composite pass

## `src/render/refractive_renderer.cpp`

Stub implementation of the three-pass renderer.

---

# Interaction Module

## `include/wd/interaction/input_router.h`

Declares `InputRouter` and `InputState`.

This is currently a GLFW-free stub. Wire actual GLFW callbacks here later.

## `src/interaction/input_router.cpp`

Updates mouse-button transition states.

## `include/wd/interaction/picker.h`

Declares `Picker`, which converts mouse coordinates to a camera ray and picks the support surface.

## `src/interaction/picker.cpp`

Placeholder implementation of camera ray generation. Replace it with inverse-view-projection unprojection.

## `include/wd/interaction/drag_interactor.h`

Declares `DragInteractor`, which converts mouse input into `DragForceField` updates.

## `src/interaction/drag_interactor.cpp`

Activates, updates, and deactivates the drag force field based on mouse input.

---

# UI Module

## `include/wd/ui/ui_controller.h`

Declares the UI controller.

## `src/ui/ui_controller.cpp`

ImGui stub. Add sliders for solver, material, render, and merge/split parameters here.

---

# Experiment Module

## `include/wd/experiments/experiment_logger.h`

Declares a CSV experiment logger.

## `src/experiments/experiment_logger.cpp`

Records and saves:

- Simulation time
- Render time
- Mean and max volume error
- Droplet count
- Total vertex count

---

# App Module

## `include/wd/app/app.h`

Declares the top-level application object.

The final application should own the scene, simulation system, renderer, input router, drag interactor, UI controller, and experiment logger.

## `src/app/app.cpp`

Creates a default scene and runs a short CPU-only smoke test. Replace the smoke test with the real GLFW/OpenGL main loop later.

## `src/main.cpp`

Executable entry point.

---

# Shader Files

## `assets/shaders/scene.vert` and `assets/shaders/scene.frag`

Placeholders for drawing the support surface and background.

## `assets/shaders/droplet_gbuffer.vert` and `assets/shaders/droplet_gbuffer.frag`

Placeholders for outputting droplet normal, depth, and thickness buffers.

## `assets/shaders/refract_composite.vert` and `assets/shaders/refract_composite.frag`

Placeholders for full-screen refractive composition.

---

# Suggested Team Split

## Simulation owner

Primary files:

- `src/sim/operators.cpp`
- `src/sim/single_droplet_solver.cpp`
- `src/sim/droplet.cpp`
- `src/sim/droplet_factory.cpp`

Tasks:

- Deformable-surface single-droplet solver
- Volume correction
- Contact line behavior
- Friction and viscosity
- Curvature-flow improvement

## Rendering owner

Primary files:

- `src/render/*`
- `assets/shaders/*`

Tasks:

- Mesh rendering
- Scene color/depth pass
- Droplet normal/depth/thickness pass
- Refractive shader
- Visual polish

## Interaction / surface owner

Primary files:

- `src/surface/*`
- `src/forces/*`
- `src/interaction/*`
- `src/ui/*`

Tasks:

- Flat plane and vertical glass presets
- Height-field surface
- Mouse dragging
- ImGui sliders
- Force-field presets

## Extensions / evaluation owner

Primary files:

- `src/sim/merge_split_controller.cpp`
- `src/experiments/experiment_logger.cpp`

Tasks:

- Simple merge
- Simple split
- CSV logging
- Performance graphs
- Volume preservation graphs
- Parameter sweep experiments
