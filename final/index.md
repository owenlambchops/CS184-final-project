<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Real-Time Lagrangian Water Droplet Simulation</title>
    <style>
        :root {
            --bg-color: #fafafa;
            --text-color: #333;
            --link-color: #2563eb;
            --border-color: #e5e7eb;
            --card-bg: #ffffff;
        }

        @media (prefers-color-scheme: dark) {
            :root {
                --bg-color: #111827;
                --text-color: #f3f4f6;
                --link-color: #60a5fa;
                --border-color: #374151;
                --card-bg: #1f2937;
            }
        }

        body {
            font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            line-height: 1.6;
            color: var(--text-color);
            background-color: var(--bg-color);
            max-width: 900px;
            margin: 0 auto;
            padding: 2rem 1rem;
        }

        header {
            text-align: center;
            margin-bottom: 3rem;
            padding-bottom: 2rem;
            border-bottom: 1px solid var(--border-color);
        }

        h1 {
            font-size: 2.25rem;
            margin-bottom: 0.5rem;
            line-height: 1.2;
        }

        .subtitle {
            font-size: 1.1rem;
            color: #6b7280;
            margin-bottom: 1rem;
        }

        .team {
            display: flex;
            justify-content: center;
            gap: 1.5rem;
            flex-wrap: wrap;
            font-weight: 500;
        }

        nav {
            display: flex;
            justify-content: center;
            gap: 1rem;
            margin-bottom: 1.5rem;
        }

        .link-strip {
            display: flex;
            justify-content: center;
            gap: 1.5rem;
            margin-bottom: 3rem;
            font-size: 0.95rem;
        }

        a {
            color: var(--link-color);
            text-decoration: none;
        }

        a:hover {
            text-decoration: underline;
        }

        .pending-link {
            color: #9ca3af;
            font-style: italic;
        }

        h2 {
            font-size: 1.8rem;
            margin-top: 2.5rem;
            margin-bottom: 1rem;
            border-bottom: 2px solid var(--border-color);
            padding-bottom: 0.3rem;
        }

        h3 {
            font-size: 1.4rem;
            margin-top: 2rem;
            margin-bottom: 0.75rem;
        }

        p {
            margin-bottom: 1rem;
        }

        ul {
            margin-bottom: 1rem;
            padding-left: 1.5rem;
        }

        li {
            margin-bottom: 0.5rem;
        }

        pre {
            background-color: var(--card-bg);
            padding: 1rem;
            border-radius: 8px;
            border: 1px solid var(--border-color);
            overflow-x: auto;
            margin-bottom: 1.5rem;
        }

        code {
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
            font-size: 0.9em;
        }

        /* Grid for side-by-side videos and images */
        .result-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 1.5rem;
            margin: 2rem 0;
        }

        .result-grid.four-cols {
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        }

        figure {
            margin: 0;
            background-color: var(--card-bg);
            border-radius: 8px;
            overflow: hidden;
            border: 1px solid var(--border-color);
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
            display: flex;
            flex-direction: column;
        }

        figure video, figure img {
            width: 100%;
            height: auto;
            display: block;
            object-fit: cover;
        }

        figcaption {
            padding: 1rem;
            font-size: 0.9rem;
            text-align: center;
            color: var(--text-color);
            border-top: 1px solid var(--border-color);
        }

        .large-figure {
            margin: 2rem 0;
        }
        
        .large-figure video {
            border-radius: 8px;
            border: 1px solid var(--border-color);
        }

    </style>
</head>
<body>

    <header>
        <h1>Real-Time Lagrangian Water Droplet Simulation</h1>
        <div class="subtitle">CS 184 Final Project Report, Spring 2026</div>
        <div class="team">
            <span>Haoyi Yu</span>
            <span>Yoko Furukawa</span>
            <span>Kevin Ying</span>
            <span>Owen Lam</span>
        </div>
    </header>

    <nav aria-label="Project pages">
        <a href="#proposal">Proposal</a>
        <a href="#milestone">Milestone</a>
        <a href="#final" aria-current="page"><strong>Final Report</strong></a>
    </nav>

    <div class="link-strip">
        <a href="https://github.com/owenlambchops/CS184-final-project" target="_blank">Repository</a>
        <a href="https://docs.google.com/presentation/d/1VsuLc1AqObqEuIxH6--GQmMYpoXd5BPMRN9lEDQG4AM/edit?usp=sharing" target="_blank">Final Presentation</a>
        <span class="pending-link">TODO: Final Video link</span>
    </div>

    <h2>Abstract</h2>
    <p>This project implements a physically inspired water droplet simulation framework based on the real-time Lagrangian deformable surface model of Zhang et al. Instead of using an expensive 3D Eulerian grid, the simulator represents each droplet as a dynamic surface mesh with per-vertex positions and velocities. Surface tension is modeled through mean curvature flow, contact-line hysteresis is approximated through advancing and receding contact angle forces, and surface friction is handled with velocity-based damping and collision response.</p>
    <p>Our implementation reproduces the core methodology needed for stable small-scale droplets: deformable surface dynamics, volume preservation, mesh merging and splitting, and real-time updates. We extend the baseline system with GPU-accelerated rendering, screen-space refraction, real-time caustic approximation, interactive droplet manipulation, editable solver parameters, and support for both planar and heightfield surfaces.</p>
    <p>The final framework is organized into five subsystems:</p>
    <ul>
        <li><strong>Simulation System:</strong> droplet physics, numerical integration, merge/split logic, and volume correction.</li>
        <li><strong>Surface Representation:</strong> analytical plane surfaces and sampled heightfield surfaces.</li>
        <li><strong>Rendering Pipeline:</strong> GPU mesh buffers, droplet cache management, refraction, caustics, and debug views.</li>
        <li><strong>Interaction System:</strong> mouse picking, drag forces, and plane tilt controls.</li>
        <li><strong>UI Layer:</strong> Dear ImGui controls for simulation, rendering, material, and solver parameters.</li>
    </ul>

    <h2>Technical Approach</h2>

    <h3>Simulation System</h3>
    <p>Each droplet is built from a rest-state mesh template. The template stores rest vertices, triangle faces, boundary loops, and topology data used by contact-line forces, volume preservation, and splitting. The default droplet mesh is generated as a spherical cap with radial rings and angular segments, which produces a dome-like shape suitable for water droplets on a surface.</p>
    <p>At runtime, each droplet maintains dynamic per-vertex positions and velocities. The <code>updateDerived()</code> step recomputes geometric quantities such as vertex normals, center of mass, average velocity, footprint radius, volume, and principal deformation axes. These derived values are used by the solver and by merge/split decisions.</p>
    <p>New droplets are created by <code>DropletFactory</code>. The factory samples the active surface, builds a local tangent frame from the surface normal, and transforms the template into world space. This allows spawned droplets to align naturally with either a flat plane or a heightfield.</p>

    <h3>Merging and Splitting</h3>
    <p>Droplet merging is triggered when two centers are sufficiently close:</p>
    <pre><code>distance < factor * combinedRadius</code></pre>
    <p>The merged droplet conserves total volume and approximates momentum conservation by blending centers and velocities with volume-weighted averages. For mesh reconstruction, we implemented a ray-cast shrinkwrap approximation instead of relying on Boolean union libraries. A spherical template is projected onto the intersecting hulls of the source droplets; rays from the new center of mass are tested against the source meshes using the Moller-Trumbore intersection algorithm, then adjusted to the farthest valid intersection.</p>
    <p>Splitting occurs when a droplet exceeds the configured volume threshold and becomes sufficiently elongated. The system spawns two smaller droplets along opposite directions of the principal deformation axis, approximating capillary breakup while staying simple enough for real-time use.</p>

    <div class="result-grid">
        <figure>
            <video src="merge/sphere_merge.mp4" autoplay loop muted playsinline></video>
            <figcaption>Zero-gravity droplet merge with a spherical template</figcaption>
        </figure>
        <figure>
            <video src="merge/rectangle_merge.mp4" autoplay loop muted playsinline></video>
            <figcaption>Droplet merge from a rectangular initialization</figcaption>
        </figure>
        <figure>
            <video src="merge/rectangle_merge_mesh.mp4" autoplay loop muted playsinline></video>
            <figcaption>Reconstructed merged mesh</figcaption>
        </figure>
    </div>

    <h3>Physics Operators</h3>
    <p>The simulation is built from modular operators that update vertex velocities and positions for different physical effects:</p>
    <ul>
        <li><strong>External forces:</strong> gravity and user-driven force fields.</li>
        <li><strong>Collision response:</strong> surface penetration correction, friction, and adhesion.</li>
        <li><strong>Viscosity:</strong> linear damping and Laplacian velocity smoothing controlled by <code>eta</code> and <code>mu</code>.</li>
        <li><strong>Surface tension:</strong> mean curvature flow using cotangent-weighted Laplacian smoothing.</li>
        <li><strong>Contact dynamics:</strong> advancing and receding contact-angle correction on boundary vertices.</li>
        <li><strong>Volume preservation:</strong> final correction pass that restores the target droplet volume.</li>
    </ul>
    <p>We use global volume correction as the default stability mechanism. Zhang et al. also describe local correction, but our experiments showed that high-frequency local adjustments often degraded stability for our mesh resolutions. Isolating volume preservation in a dedicated <code>VolumeCorrector</code> operator gave more predictable behavior than embedding volume stiffness directly in curvature flow.</p>

    <h3>Single-Droplet Solver & Zero-Gravity Analysis</h3>
    <p><code>SingleDropletSolver</code> advances each droplet in a fixed order:</p>
    <ol>
        <li>Recompute derived geometry.</li>
        <li>Apply viscosity.</li>
        <li>Apply curvature flow.</li>
        <li>Apply contact-angle forces.</li>
        <li>Apply external forces.</li>
        <li>Integrate velocities and positions.</li>
        <li>Project collisions against the active surface.</li>
        <li>Apply volume correction.</li>
    </ol>
    <p>The solver uses operator splitting so each physical effect can be enabled, disabled, and tuned independently. Below is our isolated analysis of surface tension effects in a zero-gravity environment, demonstrating stable free droplet behavior driven primarily by curvature flow.</p>

    <div class="result-grid">
        <figure>
            <video src="img:mp4/cube_zero_g/cube_0g.mp4" autoplay loop muted playsinline></video>
            <figcaption>Zero Gravity - Stable free droplet behavior</figcaption>
        </figure>
        <figure>
            <video src="img:mp4/cube_zero_g/cube_0g_mesh.mp4" autoplay loop muted playsinline></video>
            <figcaption>Zero Gravity - Corresponding Mesh Output</figcaption>
        </figure>
    </div>

    <h3>Gravity Adjustments & Resting Behavior</h3>
    <p>By tweaking external forces, we observed the resting states of the droplets under varying levels of gravitational acceleration. Notice how the droplet profile flattens out as gravitational forces begin to overpower capillary and surface tension limits.</p>
    <div class="result-grid four-cols">
        <figure>
            <video src="img:mp4/gravity/r_0.2_g_1.mp4" autoplay loop muted playsinline></video>
            <figcaption>g = 1.0 (Low Gravity)</figcaption>
        </figure>
        <figure>
            <video src="img:mp4/gravity/r_0.2_g_3.mp4" autoplay loop muted playsinline></video>
            <figcaption>g = 3.0</figcaption>
        </figure>
        <figure>
            <video src="img:mp4/gravity/r_0.2_g_5.mp4" autoplay loop muted playsinline></video>
            <figcaption>g = 5.0</figcaption>
        </figure>
        <figure>
            <video src="img:mp4/gravity/r_0.2_g_9.81.mp4" autoplay loop muted playsinline></video>
            <figcaption>g = 9.81 (Earth Gravity)</figcaption>
        </figure>
    </div>

    <h2>Surface and Mesh Representation</h2>

    <h3>Plane Surface</h3>
    <p>The plane surface backend provides analytical closest-point projection, analytical ray intersection, and exact normals. Ray-plane intersection is computed with:</p>
    <pre><code>t = (origin - ray.origin).dot(normal) / denom</code></pre>
    <p>This surface is used for the main droplet demos and for interactive plane tilt.</p>

    <h3>Heightfield Surface</h3>
    <p>The heightfield surface samples a grid of heights using bilinear interpolation. Normals are estimated from finite-difference slopes:</p>
    <pre><code>normal = normalize((-dhdx, 1, -dhdz))</code></pre>
    <p>Raycasting uses iterative signed-distance stepping:</p>
    <pre><code>t += abs(distance) * 0.5</code></pre>
    <p>This gives a generalized path for bumpy or non-flat surfaces without changing the droplet solver interface.</p>

    <h3>Mesh Data</h3>
    <p>Droplet geometry is stored as triangle meshes. Face area is computed from cross products, while lumped vertex area assigns one third of each adjacent triangle area to each vertex. These lumped areas approximate the mass matrix used by curvature flow, viscosity, and volume correction.</p>

    <h2>Rendering Pipeline</h2>

    <h3>GPU Mesh Buffers and Cache</h3>
    <p><code>GpuMeshBuffer</code> manages VAOs, VBOs, and EBOs for each droplet mesh. Positions and normals are updated every frame with dynamic buffer uploads, then rendered as triangles. <code>DropletGpuCache</code> keeps GPU buffers synchronized with the live droplet list: new droplets allocate buffers, removed droplets release them, and existing droplets update in place.</p>

    <h3>Refractive Renderer</h3>
    <p>The renderer is organized as a multi-pass OpenGL pipeline:</p>
    <ul>
        <li><strong>Light-space droplet buffer pass:</strong> render droplets from the HDRI sun direction, storing light-view normals, front depth, and back depth.</li>
        <li><strong>Caustic map pass:</strong> use light-space droplet buffers to project refracted photon splats onto the finite plane.</li>
        <li><strong>Scene pass:</strong> render the HDRI background and support plane, including projected caustics and droplet shadow.</li>
        <li><strong>Droplet G-buffer pass:</strong> store camera-view front normals and front depth.</li>
        <li><strong>Droplet back-depth pass:</strong> render back depth for screen-space thickness estimation.</li>
        <li><strong>Composite pass:</strong> combine scene color, screen-space refraction, Fresnel environment reflection, and thickness absorption.</li>
        <li><strong>Wireframe debug pass:</strong> optionally overlay mesh wireframes for solver and topology debugging.</li>
    </ul>

    <div class="result-grid">
        <figure>
            <video src="img:mp4/droplet_rendering/0.2m_droplet_fall.mp4" autoplay loop muted playsinline></video>
            <figcaption>0.2m droplet falling demo</figcaption>
        </figure>
        <figure>
            <img src="img:mp4/droplet_rendering/caustics%20and%20rafraction.png" alt="Caustic and refraction demo">
            <figcaption>Caustic and refraction demo</figcaption>
        </figure>
    </div>

    <h2>Interaction and UI</h2>
    <p>The interaction system allows users to manipulate droplets directly. <code>DragInteractor</code> builds a picking ray from the camera, raycasts against the active surface, and moves a radial drag force field to the cursor-projected world position. When dragging is active, droplets near the cursor receive external force samples from the drag field.</p>
    <p>The UI is built with Dear ImGui and exposes simulation and rendering parameters in real time:</p>
    <ul>
        <li>Debug views: final view, thickness view, normal map, caustics map, environment map, and wireframe.</li>
        <li>Droplet creation: editable spawn coordinates and interactive droplet creation.</li>
        <li>Gravity editing: live gravity vector controls.</li>
        <li>Plane tilt: pitch and roll controls from -80 to 80 degrees.</li>
        <li>Material controls: surface tension, friction, viscous damping, contact angles, and volume stiffness.</li>
        <li>Solver controls: toggles for collision, viscosity, curvature flow, contact angle simulation, and volume correction.</li>
    </ul>

    <h2>Results</h2>
    <p>Our final implementation achieves a real-time interactive droplet simulator with deformable Lagrangian surface dynamics, direct user manipulation, GPU-backed rendering, caustic/refraction effects, and modular controls for debugging and tuning. The project also includes a Python prototyping path used during earlier physics experiments and validation.</p>

    <div class="large-figure" style="text-align: center; padding: 2rem; background: var(--card-bg); border: 1px dashed var(--border-color); border-radius: 8px;">
        <strong>TODO: Final Video</strong>
        <p class="pending-link">Add the final deliverable video link here once it is available.</p>
    </div>

    <p>Key completed features include:</p>
    <ul>
        <li>Stable droplet dynamics with curvature flow, viscosity, collision, contact-angle forces, and volume correction.</li>
        <li>Real-time merging and splitting approximations using volume-aware reconstruction.</li>
        <li>Plane and heightfield surface backends under a shared surface interface.</li>
        <li>Screen-space refraction, caustic projection, HDRI environment reflection, and debug render views.</li>
        <li>Interactive controls for spawning, dragging, tilting, and tuning material/solver parameters.</li>
    </ul>

    <h2>Challenges</h2>
    <p>The main challenge was numerical stability. Early versions struggled to maintain a spherical-cap shape and often collapsed into a flat puddle or jittered under gravity. The most likely cause was an incorrect and overly coupled volume correction implementation.</p>

    <p>Zhang et al. describe both local correction, which adjusts velocity through normal components, and global correction, which offsets the surface by:</p>
    <pre><code>d = Delta V / A</code></pre>
    <p>In our initial solver, we added a volume stiffness factor directly inside curvature flow. This made the droplet artificially stiff and interfered with the geometry that surface tension was supposed to preserve. We fixed the issue by introducing a dedicated <code>VolumeCorrector</code>, applying it last, and scaling the Laplacian vertex mass more carefully.</p>

    <figure class="large-figure">
        <video src="img:mp4/smooshed_pancake.mp4" autoplay loop muted playsinline></video>
        <figcaption>Smooshed pancake bug (droplet collapsing into an unstable flat puddle)</figcaption>
    </figure>

    <p>We also found that physical scale matters. With gravity set to -9.81 in meter-second units, capillary effects become visually dominant only for small droplets, roughly in the millimeter range. At larger radii, gravity can dominate the curvature-flow response and flatten the droplet unless parameters are tuned carefully. We carefully tested the evolution of the surface behavior across varying contact angles.</p>

    <figure class="large-figure">
        <video src="img:mp4/droplet%20evolution%20for%20different%20contact%20angle.mov" autoplay loop muted playsinline></video>
        <figcaption>Droplet evolution testing for varying contact angles over time</figcaption>
    </figure>

    <div class="result-grid" style="grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));">
        <figure>
            <img src="img:mp4/contact%20angle%20range%20%5B15%2C%2045%5D.png" alt="Contact angle range [15°, 45°]">
            <figcaption>Contact angle range [15°, 45°]</figcaption>
        </figure>
        <figure>
            <img src="img:mp4/contact%20angle%20range%20%5B75%2C%20105%5D.png" alt="Contact angle range [75°, 105°]">
            <figcaption>Contact angle range [75°, 105°]</figcaption>
        </figure>
        <figure>
            <img src="img:mp4/contact%20angle%20range%20%5B160%2C%20175%5D.png" alt="Contact angle range [160°, 175°]">
            <figcaption>Contact angle range [160°, 175°]</figcaption>
        </figure>
    </div>

    <p>To improve stability, we added clamps for velocities and translations and allowed local volume correction to be disabled. The final droplet behavior is stable enough for real-time interaction, although it does not always settle perfectly at equilibrium. Because point-based surface models can retain excess energy, practical damping terms were the most effective solution for this project.</p>

    <h2>References</h2>
    <ol>
        <li>Wang, H., Mucha, P. J., and Turk, G. (2005). <em>Water Drops on Surfaces</em>. ACM SIGGRAPH 2005. <a href="https://faculty.cc.gatech.edu/~turk/my_papers/droplet.pdf" target="_blank">PDF</a></li>
        <li>El-Ajou, A., et al. (2019). <em>Mathematical Model for Simulating the Movement of Water Droplet on Artificial Leaf Surface</em>. Frontiers in Physics, 7:132. <a href="https://www.frontiersin.org/journals/physics/articles/10.3389/fphy.2019.00132/full" target="_blank">DOI</a></li>
        <li>Saye, R. I., and Sethian, J. A. (2013). <em>Multiscale Modeling of Membrane Rearrangement, Drainage, and Rupture in Evolving Foams</em>. Science, 340, 720-724.</li>
        <li>Zhang, Y., Wang, H., Wang, S., Tong, Y., and Zhou, K. (2012). <em>A Deformable Surface Model for Real-Time Water Drop Animation</em>. IEEE Transactions on Visualization and Computer Graphics, 18(8), 1281-1289. <a href="https://doi.org/10.1109/TVCG.2011.141" target="_blank">DOI</a></li>
        <li>Iwasaki, K., Matsuzawa, K., and Nishita, T. (2004). <em>Real-time rendering of soap bubbles taking into account light interference</em>. Proceedings Computer Graphics International, 344-348. <a href="https://ieeexplore.ieee.org/document/1309231" target="_blank">DOI</a></li>
        <li>GPU Gems 2, Chapter 19. <em>Generic Refraction Simulation</em>. NVIDIA Developer. <a href="https://developer.nvidia.com/gpugems/gpugems2/part-ii-shading-lighting-and-shadows/chapter-19-generic-refraction-simulation" target="_blank">Article</a></li>
        <li>Simon, G. (2010). <em>Screen Space Fluid Rendering for Games</em>. NVIDIA Direct3D Effects Presentation. <a href="https://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf" target="_blank">PDF</a></li>
        <li>ShaderToy references: <a href="https://www.shadertoy.com/view/wlyfWw" target="_blank">Water Droplets Shader 1</a> and <a href="https://www.shadertoy.com/view/4ll3R7" target="_blank">Water Droplets Shader 2</a>.</li>
    </ol>

    <h2>Contributions</h2>
    <ul>
        <li><strong>Simulation System:</strong> Kevin, Owen, Haoyi</li>
        <li><strong>Plane Representation:</strong> Haoyi, Kevin, Yoko</li>
        <li><strong>Mesh Representation:</strong> Kevin, Haoyi, Owen</li>
        <li><strong>Mesh Operations:</strong> Kevin, Owen, Haoyi</li>
        <li><strong>UI Interface:</strong> Haoyi, Kevin, Yoko</li>
        <li><strong>Rendering Pipeline:</strong> Haoyi, Kevin</li>
        <li><strong>Plane Cursor Interaction:</strong> Kevin, Yoko</li>
        <li><strong>Writeup:</strong> Yoko, Kevin, Haoyi, Owen</li>
    </ul>

</body>
</html>