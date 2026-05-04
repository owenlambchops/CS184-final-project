## Plan: Port droplet simulation to C++

Goal: move the functioning Python droplet simulation behavior into the existing C++ simulation stack under `water_droplet_sim_skeleton/src/sim`, using the current solver/operator architecture rather than building a second simulation path. The Python code stays as a reference implementation for comparison while the C++ path becomes the primary runtime.

**Steps**
1. Map the Python simulation state and parameters onto the existing C++ data model, using `Droplet`, `MaterialParams`, `SolverParams`, and `SimulationSystem` as the owning abstractions. Confirm which Python knobs belong in per-droplet material state versus global solver state, and preserve the current public simulation entry point in `SingleDropletSolver`.
2. Port the Python physics helpers into the C++ sim layer in a C++-native way, reusing the current operator split where possible. The key behaviors to preserve are cotangent-weight Laplacians, vertex-normal and volume computation, boundary/contact-angle hysteresis, collision/friction handling, and the damping pipeline used in `update_forward_euler` and `update_velocity_verlet`.
3. Rework `SingleDropletSolver::step` so it executes the same control flow as the chosen Python update path, including substeps, update ordering, velocity clipping, collision projection, and optional volume correction. If Verlet support is kept, make its branching explicit and ensure the returned state ordering is consistent across both modes.
4. Align droplet creation and initialization so spawned droplets start from the same assumptions as Python, including target volume, initial velocity, surface anchoring, and default contact/friction parameters. Reuse `DropletFactory` and the template-based mesh initialization rather than introducing a parallel construction path.
5. Reconcile any parameter semantics that are currently implicit or duplicated in Python, especially the meanings of `eta`, `mu`, `gamma`, `k_v`, `boundary_alpha`, adhesion distance, and velocity damping limits. Put the canonical values and ranges in the C++ parameter structs and, if needed, the UI/controller layer.
6. Keep the Python implementation available as a reference harness while the C++ port lands, so behavior can be compared against the prototype when questions arise. Do not change rendering or merge/split behavior unless a solver-side change requires it.

**Relevant files**
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/python_sim_dev/water_sim_basic.py` — source of the reference physics and update ordering.
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/include/wd/core/types.h` — `MaterialParams`, `SolverParams`, and shared simulation types.
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/include/wd/sim/droplet.h` and `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/src/sim/droplet.cpp` — droplet state, derived quantities, and geometry ownership.
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/include/wd/sim/operators.h` and `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/src/sim/operators.cpp` — existing physics operators to extend or adjust.
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/include/wd/sim/single_droplet_solver.h` and `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/src/sim/single_droplet_solver.cpp` — the main integration loop to refactor.
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/src/sim/droplet_factory.cpp` — droplet spawning and initialization.
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/src/sim/simulation_system.cpp` — orchestration and post-step bookkeeping.
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/src/app/app.cpp` and `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/water_droplet_sim_skeleton/include/wd/ui/ui_controller.h` — only if solver parameters need to be surfaced in the UI.

**Verification**
1. Build the C++ project and confirm the sim target compiles cleanly after each solver/operator change.
2. Rebuild after wiring any new parameters or signatures to ensure the public simulation path still links.
3. Use the Python prototype only as a reference during implementation, not as the primary verification gate, since the chosen validation level is build-only.

**Decisions**
- The port should be C++-native, not a line-by-line wrapper around Python.
- The Python implementation stays in the repo as a reference implementation while the C++ path is being completed.
- Verification scope is build/compile focused rather than full numeric parity testing.
- Existing rendering and merge/split behavior are out of scope unless the solver port forces a direct dependency.
