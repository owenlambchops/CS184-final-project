---
layout: project
title: "Real-Time Lagrangian Water Droplet Simulation"
date: CS 184 Final Project Report, Spring 2026
team:
  - Haoyi Yu
  - Yoko Furukawa
  - Kevin Ying
  - Owen Lam
---

<nav class="page-nav" aria-label="Project pages">
  <a href="{{ '/' | relative_url }}">Proposal</a>
  <a href="{{ '/milestone/' | relative_url }}">Milestone</a>
  <a href="{{ '/final/' | relative_url }}" aria-current="page">Final Report</a>
</nav>

<div class="link-strip">
  <a href="https://github.com/owenlambchops/CS184-final-project" target="_blank">Repository</a>
  <a href="https://docs.google.com/presentation/d/1VsuLc1AqObqEuIxH6--GQmMYpoXd5BPMRN9lEDQG4AM/edit?usp=sharing" target="_blank">Final Presentation</a>
  <a href="https://drive.google.com/file/d/13uo6rrP5mZ7I9dr6Eewg-8rnYVzrKPFJ/view?usp=drive_link" target="_blank">Final Video</a>
</div>

## Abstract

This project implements a physically inspired water droplet simulation framework based on the real-time Lagrangian deformable surface model of Zhang et al. Instead of using an expensive 3D Eulerian grid, the simulator represents each droplet as a dynamic surface mesh with per-vertex positions and velocities. Surface tension is modeled through mean curvature flow, contact-line hysteresis is approximated through advancing and receding contact angle forces, and surface friction is handled with velocity-based damping and collision response.

Our implementation reproduces the core methodology needed for stable small-scale droplets: deformable surface dynamics, volume preservation, mesh merging and splitting, and real-time updates. We extend the baseline system with GPU-accelerated rendering, screen-space refraction, real-time caustic approximation, interactive droplet manipulation, editable solver parameters, and support for both planar and heightfield surfaces.

The final framework is organized into five subsystems:

- **Simulation System:** droplet physics, numerical integration, merge/split logic, and volume correction.
- **Surface Representation:** analytical plane surfaces and sampled heightfield surfaces.
- **Rendering Pipeline:** GPU mesh buffers, droplet cache management, refraction, caustics, and debug views.
- **Interaction System:** mouse picking, drag forces, and plane tilt controls.
- **UI Layer:** Dear ImGui controls for simulation, rendering, material, and solver parameters.

## Technical Approach

### Simulation System

Each droplet is built from a rest-state mesh template. The template stores rest vertices, triangle faces, boundary loops, and topology data used by contact-line forces, volume preservation, and splitting. The default droplet mesh is generated as a spherical cap with radial rings and angular segments, which produces a dome-like shape suitable for water droplets on a surface.

At runtime, each droplet maintains dynamic per-vertex positions and velocities. The `updateDerived()` step recomputes geometric quantities such as vertex normals, center of mass, average velocity, footprint radius, volume, and principal deformation axes. These derived values are used by the solver and by merge/split decisions.

New droplets are created by `DropletFactory`. The factory samples the active surface, builds a local tangent frame from the surface normal, and transforms the template into world space. This allows spawned droplets to align naturally with either a flat plane or a heightfield.

### Merging and Splitting

Droplet merging is triggered when two centers are sufficiently close:

```text
distance < factor * combinedRadius
```

The merged droplet conserves total volume and approximates momentum conservation by blending centers and velocities with volume-weighted averages. For mesh reconstruction, we implemented a ray-cast shrinkwrap approximation instead of relying on Boolean union libraries. A spherical template is projected onto the intersecting hulls of the source droplets; rays from the new center of mass are tested against the source meshes using the Moller-Trumbore intersection algorithm, then adjusted to the farthest valid intersection.

Splitting occurs when a droplet exceeds the configured volume threshold and becomes sufficiently elongated. The system spawns two smaller droplets along opposite directions of the principal deformation axis, approximating capillary breakup while staying simple enough for real-time use.

<div class="result-grid">
  <a class="result-card" href="https://drive.google.com/file/d/1WU8ETLVTez5zuLFc7gsLda-OXvNIKPNJ/view?usp=drive_link" target="_blank">
    <span>Merge Result</span>
    Zero-gravity droplet merge with a spherical template
  </a>
  <a class="result-card" href="https://drive.google.com/file/d/19-BSAl8hBH7hrk6hsECibiW3SPXyxHCi/view?usp=drive_link" target="_blank">
    <span>Merge Result</span>
    Droplet merge from a rectangular initialization
  </a>
  <a class="result-card" href="https://drive.google.com/file/d/134hfNsUMwXVeo9SRZVCSdNgp7sTXVaIw/view?usp=drive_link" target="_blank">
    <span>Mesh Output</span>
    Reconstructed merged mesh
  </a>
</div>

### Physics Operators

The simulation is built from modular operators that update vertex velocities and positions for different physical effects:

- **External forces:** gravity and user-driven force fields.
- **Collision response:** surface penetration correction, friction, and adhesion.
- **Viscosity:** linear damping and Laplacian velocity smoothing controlled by `eta` and `mu`.
- **Surface tension:** mean curvature flow using cotangent-weighted Laplacian smoothing.
- **Contact dynamics:** advancing and receding contact-angle correction on boundary vertices.
- **Volume preservation:** final correction pass that restores the target droplet volume.

We use global volume correction as the default stability mechanism. Zhang et al. also describe local correction, but our experiments showed that high-frequency local adjustments often degraded stability for our mesh resolutions. Isolating volume preservation in a dedicated `VolumeCorrector` operator gave more predictable behavior than embedding volume stiffness directly in curvature flow.

### Single-Droplet Solver

`SingleDropletSolver` advances each droplet in a fixed order:

1. Recompute derived geometry.
2. Apply viscosity.
3. Apply curvature flow.
4. Apply contact-angle forces.
5. Apply external forces.
6. Integrate velocities and positions.
7. Project collisions against the active surface.
8. Apply volume correction.

The solver uses operator splitting so each physical effect can be enabled, disabled, and tuned independently. The original baseline uses semi-implicit Euler integration for simplicity and speed. We also explored velocity Verlet integration as a higher-performance and more stable alternative for particle-like dynamics.

<div class="result-grid">
  <a class="result-card" href="https://drive.google.com/drive/folders/16xBZ5PYXH6qWOp5wQcRlJZ7S0CRc1s1X?usp=drive_link" target="_blank">
    <span>Physics Results</span>
    Gravity simulation clips
  </a>
  <a class="result-card" href="https://drive.google.com/file/d/14r2x0epy9ZXiFBAoryr6zcY2jVFiVdJD/view?usp=drive_link" target="_blank">
    <span>Zero Gravity</span>
    Stable free droplet behavior
  </a>
  <a class="result-card" href="https://drive.google.com/file/d/1lhPp8FjWeeJfqDjUXHDc_W1gO1rrxLtw/view?usp=drive_link" target="_blank">
    <span>Mesh Output</span>
    Zero-gravity droplet mesh
  </a>
</div>

## Surface and Mesh Representation

### Plane Surface

The plane surface backend provides analytical closest-point projection, analytical ray intersection, and exact normals. Ray-plane intersection is computed with:

```text
t = (origin - ray.origin).dot(normal) / denom
```

This surface is used for the main droplet demos and for interactive plane tilt.

### Heightfield Surface

The heightfield surface samples a grid of heights using bilinear interpolation. Normals are estimated from finite-difference slopes:

```text
normal = normalize((-dhdx, 1, -dhdz))
```

Raycasting uses iterative signed-distance stepping:

```text
t += abs(distance) * 0.5
```

This gives a generalized path for bumpy or non-flat surfaces without changing the droplet solver interface.

### Mesh Data

Droplet geometry is stored as triangle meshes. Face area is computed from cross products, while lumped vertex area assigns one third of each adjacent triangle area to each vertex. These lumped areas approximate the mass matrix used by curvature flow, viscosity, and volume correction.

## Rendering Pipeline

### GPU Mesh Buffers and Cache

`GpuMeshBuffer` manages VAOs, VBOs, and EBOs for each droplet mesh. Positions and normals are updated every frame with dynamic buffer uploads, then rendered as triangles. `DropletGpuCache` keeps GPU buffers synchronized with the live droplet list: new droplets allocate buffers, removed droplets release them, and existing droplets update in place.

### Refractive Renderer

The renderer is organized as a multi-pass OpenGL pipeline:

- **Light-space droplet buffer pass:** render droplets from the HDRI sun direction, storing light-view normals, front depth, and back depth.
- **Caustic map pass:** use light-space droplet buffers to project refracted photon splats onto the finite plane.
- **Scene pass:** render the HDRI background and support plane, including projected caustics and droplet shadow.
- **Droplet G-buffer pass:** store camera-view front normals and front depth.
- **Droplet back-depth pass:** render back depth for screen-space thickness estimation.
- **Composite pass:** combine scene color, screen-space refraction, Fresnel environment reflection, and thickness absorption.
- **Wireframe debug pass:** optionally overlay mesh wireframes for solver and topology debugging.

<div class="render-gallery">
  <a class="render-card" href="https://drive.google.com/file/d/1CvC11SIBHfAknFvKy9xjTrbiT1FwV8EY/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/caustics.png' | relative_url }}" alt="Caustics render output cover">
    <div class="render-card-body">
      <span>Caustic Map</span>
      <strong>Caustics</strong>
    </div>
  </a>
  <a class="render-card" href="https://drive.google.com/file/d/1w-Aik9zMIkhAMyp4cwLxxnV0Y9a0pKzF/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/droplet_depth.png' | relative_url }}" alt="Droplet depth buffer cover">
    <div class="render-card-body">
      <span>Droplet Buffer</span>
      <strong>Droplet Depth</strong>
    </div>
  </a>
  <a class="render-card" href="https://drive.google.com/file/d/1Sh-dN5xsbPD_rWdgxdylHHBeut2IvkBW/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/droplet_normal.png' | relative_url }}" alt="Droplet normal buffer cover">
    <div class="render-card-body">
      <span>Droplet Buffer</span>
      <strong>Droplet Normal</strong>
    </div>
  </a>
  <a class="render-card" href="https://drive.google.com/file/d/1ECn6pIfrYndvJrvz7d1sPIi75-0eKIwp/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/final.png' | relative_url }}" alt="Final refractive renderer output cover">
    <div class="render-card-body">
      <span>Composite</span>
      <strong>Final</strong>
    </div>
  </a>
  <a class="render-card" href="https://drive.google.com/file/d/1wHsI9ntVja8CLxKPFhNUz4FAXE0QJTiW/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/scene_color.png' | relative_url }}" alt="Scene color buffer cover">
    <div class="render-card-body">
      <span>Scene Buffer</span>
      <strong>Scene Color</strong>
    </div>
  </a>
  <a class="render-card" href="https://drive.google.com/file/d/1jmHEvQ3FXxXkLphQDi4R9tcyQt3UsA_f/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/scene_depth.png' | relative_url }}" alt="Scene depth buffer cover">
    <div class="render-card-body">
      <span>Scene Buffer</span>
      <strong>Scene Depth</strong>
    </div>
  </a>
  <a class="render-card" href="https://drive.google.com/file/d/1L3ayBmNnyazHqolwDqC8jArRsFLjKdIc/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/thickness.png' | relative_url }}" alt="Droplet thickness buffer cover">
    <div class="render-card-body">
      <span>Composite Input</span>
      <strong>Thickness</strong>
    </div>
  </a>
  <a class="render-card" href="https://drive.google.com/file/d/1cmZBOTKpatGR12bCpoUKvLcwxppx72DD/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/wireframe.png' | relative_url }}" alt="Wireframe debug render cover">
    <div class="render-card-body">
      <span>Debug View</span>
      <strong>Wireframe</strong>
    </div>
  </a>
  <a class="render-card" href="https://drive.google.com/file/d/12sl18-27Mm4KWY6oTS1aLBJjiQgbbfDC/view?usp=drive_link" target="_blank">
    <img src="{{ '/final/assets/refractive/caustics_and_refraction.png' | relative_url }}" alt="Caustic and refraction demo cover">
    <div class="render-card-body">
      <span>Rendering Result</span>
      <strong>Caustic and Refraction Demo</strong>
    </div>
  </a>
</div>

## Interaction and UI

The interaction system allows users to manipulate droplets directly. `DragInteractor` builds a picking ray from the camera, raycasts against the active surface, and moves a radial drag force field to the cursor-projected world position. When dragging is active, droplets near the cursor receive external force samples from the drag field.

The UI is built with Dear ImGui and exposes simulation and rendering parameters in real time:

- Debug views: final view, thickness view, normal map, caustics map, environment map, and wireframe.
- Droplet creation: editable spawn coordinates and interactive droplet creation.
- Gravity editing: live gravity vector controls.
- Plane tilt: pitch and roll controls from -80 to 80 degrees.
- Material controls: surface tension, friction, viscous damping, contact angles, and volume stiffness.
- Solver controls: toggles for collision, viscosity, curvature flow, contact angle simulation, and volume correction.

## Results

Our final implementation achieves a real-time interactive droplet simulator with deformable Lagrangian surface dynamics, direct user manipulation, GPU-backed rendering, caustic/refraction effects, and modular controls for debugging and tuning. The project also includes a Python prototyping path used during earlier physics experiments and validation.

<div class="video-frame">
  <iframe
    src="https://drive.google.com/file/d/13uo6rrP5mZ7I9dr6Eewg-8rnYVzrKPFJ/preview"
    title="Final project video"
    allow="autoplay; encrypted-media"
    allowfullscreen>
  </iframe>
</div>

<p class="media-link">
  <a href="https://drive.google.com/file/d/13uo6rrP5mZ7I9dr6Eewg-8rnYVzrKPFJ/view?usp=drive_link" target="_blank">Open the final video on Google Drive</a>
</p>

Key completed features include:

- Stable droplet dynamics with curvature flow, viscosity, collision, contact-angle forces, and volume correction.
- Real-time merging and splitting approximations using volume-aware reconstruction.
- Plane and heightfield surface backends under a shared surface interface.
- Screen-space refraction, caustic projection, HDRI environment reflection, and debug render views.
- Interactive controls for spawning, dragging, tilting, and tuning material/solver parameters.

## Challenges

The main challenge was numerical stability. Early versions struggled to maintain a spherical-cap shape and often collapsed into a flat puddle or jittered under gravity. The most likely cause was an incorrect and overly coupled volume correction implementation.

Zhang et al. describe both local correction, which adjusts velocity through normal components, and global correction, which offsets the surface by:

```text
d = Delta V / A
```

In our initial solver, we added a volume stiffness factor directly inside curvature flow. This made the droplet artificially stiff and interfered with the geometry that surface tension was supposed to preserve. We fixed the issue by introducing a dedicated `VolumeCorrector`, applying it last, and scaling the Laplacian vertex mass more carefully.

<div class="media-placeholder">
  <strong>TODO: 0.3m droplet pancake photo</strong>
  <p>Add the image link for the unstable flattened droplet case discussed in the report.</p>
</div>

Debugging the $\texttt{ContactLineOperator}$ (based off of Zhang et al. (2012) contact angle hysteresis model), as we found it was having negligible effect due to an undersized force coefficient $\texttt{contactStiffness}$ and an adhesion zone too narrow to capture contact-line vertices. We resolved this by raising $\texttt{contactStiffness}$ and $\texttt{adhesionDistance}$ to more suitable ranges, and adding a damping term $\texttt{contactLineDamping}$ to suppress contact-line oscillation.

<div class="image-row">
  <figure>
    <img src="./img:mp4/contact%20angle%20range%20%5B15%2C%2045%5D.png" alt="Contact angle range [15°, 45°]">
    <figcaption>Contact angle range [15°, 45°]</figcaption>
  </figure>
  <figure>
    <img src="./img:mp4/contact%20angle%20range%20%5B75%2C%20105%5D.png" alt="Contact angle range [75°, 105°]">
    <figcaption>Contact angle range [75°, 105°]</figcaption>
  </figure>
  <figure>
    <img src="./img:mp4/contact%20angle%20range%20%5B160%2C%20175%5D.png" alt="Contact angle range [160°, 175°]">
    <figcaption>Contact angle range [160°, 175°]</figcaption>
  </figure>
</div>

We also found that physical scale matters. With gravity set to -9.81 in meter-second units, capillary effects become visually dominant only for small droplets, roughly in the millimeter range. At larger radii, gravity can dominate the curvature-flow response and flatten the droplet unless parameters are tuned carefully. With a much smaller radius, we found that the droplet showed a more visible boundary angle, but became more underdamped. By reducing gravity, we were able to allow the surface tension to pull the droplet into its final shape.

To improve stability, we added clamps for velocities and translations and allowed local volume correction to be disabled. The final droplet behavior is stable enough for real-time interaction, although it does not always settle perfectly at equilibrium. Because point-based surface models can retain excess energy, practical damping terms were the most effective solution for this project.

## References

1. Wang, H., Mucha, P. J., and Turk, G. (2005). _Water Drops on Surfaces_. ACM SIGGRAPH 2005. [PDF](https://faculty.cc.gatech.edu/~turk/my_papers/droplet.pdf){:target="\_blank"}
2. El-Ajou, A., et al. (2019). _Mathematical Model for Simulating the Movement of Water Droplet on Artificial Leaf Surface_. Frontiers in Physics, 7:132. [DOI](https://www.frontiersin.org/journals/physics/articles/10.3389/fphy.2019.00132/full){:target="\_blank"}
3. Saye, R. I., and Sethian, J. A. (2013). _Multiscale Modeling of Membrane Rearrangement, Drainage, and Rupture in Evolving Foams_. Science, 340, 720-724.
4. Zhang, Y., Wang, H., Wang, S., Tong, Y., and Zhou, K. (2012). _A Deformable Surface Model for Real-Time Water Drop Animation_. IEEE Transactions on Visualization and Computer Graphics, 18(8), 1281-1289. [DOI](https://doi.org/10.1109/TVCG.2011.141){:target="\_blank"}
5. Iwasaki, K., Matsuzawa, K., and Nishita, T. (2004). _Real-time rendering of soap bubbles taking into account light interference_. Proceedings Computer Graphics International, 344-348. [DOI](https://ieeexplore.ieee.org/document/1309231){:target="\_blank"}
6. GPU Gems 2, Chapter 19. _Generic Refraction Simulation_. NVIDIA Developer. [Article](https://developer.nvidia.com/gpugems/gpugems2/part-ii-shading-lighting-and-shadows/chapter-19-generic-refraction-simulation){:target="\_blank"}
7. Simon, G. (2010). _Screen Space Fluid Rendering for Games_. NVIDIA Direct3D Effects Presentation. [PDF](https://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf){:target="\_blank"}
8. ShaderToy references: [Water Droplets Shader 1](https://www.shadertoy.com/view/wlyfWw){:target="\_blank"} and [Water Droplets Shader 2](https://www.shadertoy.com/view/4ll3R7){:target="\_blank"}.

## Contributions

- **Simulation System:** Kevin, Owen, Haoyi
- **Plane Representation:** Haoyi, Kevin, Yoko
- **Mesh Representation:** Kevin, Haoyi, Owen
- **Mesh Operations:** Kevin, Owen, Haoyi
- **UI Interface:** Haoyi, Kevin, Yoko
- **Rendering Pipeline:** Haoyi, Kevin
- **Plane Cursor Interaction:** Kevin, Yoko
- **Writeup:** Yoko, Kevin, Haoyi, Owen
