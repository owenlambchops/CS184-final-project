---
layout: project
title: "Real Time Interactive Water Droplets"
team:
  - Haoyi Yu
  - Yoko Furukawa
  - Kevin Ying
  - Owen Lam
---

**Repo:** [https://github.com/owenlambchops/CS184-final-project](https://github.com/owenlambchops/CS184-final-project){:target="_blank"}

**Website:** [https://owenlambchops.github.io/CS184-final-project/](https://owenlambchops.github.io/CS184-final-project/){:target="_blank"}

## Project Summary

We plan to build a real-time interactive water droplet simulator that simulates droplet movement on a vertical glass surface without gravity as the baseline. The system emphasizes realistic visual appearance through real-time refractive shading. Aspirational extensions include physically accurate merging/splitting, caustics, customizable fluid properties like viscosity and friction, and support for more complex surfaces than a flat plane.

## Problem Description

Fluid simulation has been quite an active area of research in computer graphics. We want to focus on water droplet simulation, including real-time fluid simulation (in zero gravity and with gravity), merging/splitting, and fluid-surface contact. We reference some existing works <a class="ref" href="#ref1">[1]</a> <a class="ref" href="#ref2">[2]</a>.

## Goals and Deliverables

### Realistic Goals (Baseline)

- **Real-Time Droplet Simulation:** Deliver a real-time water droplet running simulation featuring realistic sliding and deformation behavior, reacting dynamically to external forces or changing environments (e.g., virtual tilt or wind).
- **Spec/Detail:** The core physics (surface tension, adhesion, movement) will be implemented using CPU-based C++ logic. The droplet will be modeled as a deformable polyhedron using vertex dynamics or a similar approximation method to calculate physical behavior.
- **Interactive System and Environments:** Implement an interactive screen where users can drag and interact with water droplets and observe their behavior in two distinct environments:
  - **Zero Gravity:** Droplets maintain a near-spherical shape and react primarily to external user input.
  - **Vertical Glass Surface:** Droplets exhibit movement (e.g., simulating a car window) governed by realistic friction and surface adhesion models.
- **Real-Time Refractive Shading:** Achieve realistic visual appearance through real-time refractive shading. *Spec/Detail:* Optical effects will be implemented using GLSL shaders within an OpenGL 4.1 framework, using environment mapping and screen-space normal approximations for convincing refraction and reflection (referencing ShaderToy models for visual quality).

### Aspiration Goals (Extensions)

- **Physically Accurate Topology Changes:** Implement physically accurate fluid merging and splitting functionality. *Spec/Detail:* This requires implementing advanced multiscale logic to handle structural membrane changes and topological restructuring during merging/splitting events, referencing work such as <a class="ref" href="#ref1">[1]</a> or <a class="ref" href="#ref3">[3]</a>.
- **Advanced Optical Effects:** Implement real-time caustics, specifically the light patterns cast by the refractive droplet. *Spec/Detail:* Explore optimized screen-space caustics approximations, as traditional global illumination methods may be computationally intensive for real-time performance within the OpenGL 4.1 constraints.
- **Customizable Fluid Parameters:** Allow user adjustment of custom fluid viscosity and fluid-surface friction properties. *Spec/Detail:* Implement these properties as adjustable parameters that directly influence the surface tension and adhesion terms within the core C++ vertex dynamics physics model.
- **Complex Surface Support:** Support simulation on custom, non-flat surfaces. *Spec/Detail:* Integrate support for complex 3D surfaces (e.g., bumpy terrains or artificial leaves) using pre-generated normal maps and displacement fields, requiring a generalized model of droplet traversal (referencing <a class="ref" href="#ref2">[2]</a>).

## Quantification and Validation Methods

- **Performance Targets (Real-Time):**
  - **Sustained Framerate:** The simulation-shader pipeline should maintain a stable framerate >= 30 FPS with GPU acceleration. <a class="ref" href="#ref4">[4]</a> achieved 30FPS with 50k triangles on a similar glass panel simulation.
  - **Interactive Response Time:** User interface interaction (dragging/dropping) must exhibit a response time delay < 100 ms to ensure a feeling of real-time responsiveness.
  - **Modeling Method Validation:** The chosen droplet modeling method (polyhedron or Finite Element Method (FEM)) will be benchmarked to ensure it simulates physically accurate droplet behavior while adhering to the specified performance metrics.
- **Validation Method (Empirical Accuracy):**
  - **Visual Comparison:** We will generate videos of the simulation and perform visual comparison against real-world footage of water droplets in free fall and sliding on vertical surfaces.
  - **Physical Metrics:** Quantify and compare simulated physical behaviors, such as sliding velocity, droplet deformation profiles, and contact angle stability, against theoretical models and experimental video data.

## Schedule

- **Apr 5-11:** Finish up HW4, determine libraries, review papers, set up sim environment
- **Apr 12-18:** Work on the realistic goals, deliver real-time water droplet running simulation with realistic behaviour, react to changing environments in real time. Set up an interactive interface where you can drag and interact with water droplets in zero gravity and on a vertical glass surface (e.g., car window). Real-time refractive shading ([ShaderToy](https://www.shadertoy.com/view/wlyfWw){:target="_blank"}).
- **Apr 20 Project Milestone:** Working simulation environment, correct water drop behaviour, basic functioning interactive UI.
- **Apr 19-25:** Complete unfinished realistic goal components, start drafting project report, start ahead on the aspiration goals: physically accurate fluid merging and splitting, real-time caustics.
- **Apr 26-May 2:** Writing project report, work on the aspiration goals: physically accurate fluid merging and splitting, real-time caustics, custom fluid viscosity (as opposed to just water simulation) and fluid-surface friction, custom surface rather than just flat plane. Finalize deliverable video/website.
- **May 4:** Deliverable deadline
- **May 5:** Presentation

## Resources

### 1. Computing Platform, Hardware, and Software

- The project will be developed and deployed across a mixed-OS environment, necessitating a carefully planned cross-platform framework strategy.
- **Hardware Environments:** Instruction Linux Lab machine (Cory Hall, Room 330) equipped with an NVIDIA T1000 8GB GPU. Apple MacBook Pro featuring an M1 Pro chip.
- **Operating Systems:** Ubuntu 22.04 LTS and macOS.
- **Core Implementation Framework (C++ & OpenGL):** The primary rendering and simulation pipeline will be implemented in C++ using OpenGL 4.1. This specific version constraint is required to maintain compatibility with Apple's M1 architecture, which caps OpenGL support at version 4.1 and lacks native support for OpenGL Compute Shaders. Because of this, the core physics calculations (e.g., droplet merging, gravity) will be handled on the CPU via C++, while GLSL shaders will manage real-time optical effects like refraction, reflection, and vertex deformation. Cross-platform libraries such as GLFW and GLM will be utilized to ensure seamless compilation across both Ubuntu and macOS.
- **Prototyping & Asset Generation (Python & Blender):** Python will be used extensively for mathematical prototyping. Complex fluid dynamics logic (such as viscosity and surface tension calculations) will be modeled, visualized, and debugged in Python before being ported to the main C++ codebase. Additionally, Blender can be used to model environmental assets (e.g., artificial leaves, bumpy terrains), and to generate baked normal maps or reference offline fluid simulations.

### 2. Existing Code & Systems

- Initial research into real-time optical effects, environmental reflections, and baseline movement logic is informed by the following existing shader implementations:
  - ShaderToy Reference 1: Water Droplets Shader ([ShaderToy](https://www.shadertoy.com/view/wlyfWw){:target="_blank"})
  - ShaderToy Reference 2: Water Droplets Shader ([ShaderToy](https://www.shadertoy.com/view/4ll3R7){:target="_blank"})

### 3. Reference Materials

<ol class="references">
<li id="ref1">Wang, H., Mucha, P. J., &amp; Turk, G. (2005). Water Drops on Surfaces. ACM SIGGRAPH 2005. <br><em>Application:</em> Core fluid dynamics, specifically modeling viscosity, surface tension, and the mechanics of droplet merging. <a href="https://www.cc.gatech.edu/~turk/my_papers/waterdrops.pdf" target="_blank">PDF</a></li>
<li id="ref2">El-Ajou, A., et al. (2019). Mathematical Model for Simulating the Movement of Water Droplet on Artificial Leaf Surface. Frontiers in Physics, 7:132. <br><em>Application:</em> Mathematical modeling for droplet traversal across non-flat, textured surfaces (e.g., sliding down a bumpy leaf). <a href="https://doi.org/10.3389/fphy.2019.00132" target="_blank">DOI: 10.3389/fphy.2019.00132</a></li>
<li id="ref3">Saye, R. I., &amp; Sethian, J. A. (2013). Multiscale Modeling of Membrane Rearrangement, Drainage, and Rupture in Evolving Foams. Science, 340, 720-724. <br><em>Application:</em> Advanced multiscale logic for handling structural membrane changes, relevant for rendering complex droplet intersections. <a href="https://www.science.org/doi/10.1126/science.1230623" target="_blank">DOI: 10.1126/science.1230623</a></li>
<li id="ref4">Zhang, Y., Wang, H., Wang, S., Tong, Y., &amp; Zhou, K. (2012). A Deformable Surface Model for Real-Time Water Drop Animation. IEEE Transactions on Visualization and Computer Graphics, 18(8), 1281-1289. <br><em>Application:</em> Implementing the real-time generation and animation of deformable water droplet surfaces. <a href="https://doi.org/10.1109/TVCG.2011.141" target="_blank">DOI: 10.1109/TVCG.2011.141</a></li>
<li id="ref5">Iwasaki, K., Matsuzawa, K., &amp; Nishita, T. (2004). Real-time rendering of soap bubbles taking into account light interference. Proceedings Computer Graphics International, 344-348. <br><em>Application:</em> Vertex-based approaches for modeling spherical liquid bodies and techniques for rendering complex light interference and refraction. <a href="https://doi.org/10.1109/CGI.2004.1309231" target="_blank">DOI: 10.1109/CGI.2004.1309231</a></li>
<li id="ref6">Physics Stack Exchange. <br><em>Application:</em> Consultation of community discussions regarding the underlying physics governing the resting shape of a water drop. <a href="https://physics.stackexchange.com/questions/why-are-water-droplets-spherical" target="_blank">Discussion</a></li>
</ol>
