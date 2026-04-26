## Plan: Stabilize Clean-Based Droplet Step

Apply a minimal, paper-aligned cleanup to `water_sim_basic_clean_based.py` that removes conflicting velocity feedback loops and redundant clamps causing expand-then-collapse oscillation, while keeping the current implicit MCF + volume-correction architecture intact.

**Steps**
1. **Phase 1: Isolate instability sources (blocking)**
1. Confirm and document the three interacting causes in `step(...)`: high correction-to-velocity feedback (`mcf_velocity_feedback`, `global_velocity_feedback`), redundant speed clamping (pre- and post-projection), and aggressive iterative global volume correction step cap (`d_max`) that can over-correct shape each substep.
2. Lock scope to code-only changes in `python_sim_dev/water_sim_basic_clean_based.py` (no notebook edits) per user decision.

2. **Phase 2: Minimal algorithm simplification (depends on Phase 1)**
1. In `step(...)`, remove the early speed clamp immediately after explicit gravity+viscosity update, and keep only one final safety speed clamp at end of each internal substep (single source of truth for velocity limiting).
2. Reduce correction feedback gains to avoid energy injection/limit cycles:
   - Lower `mcf_velocity_feedback` from 0.25 to a conservative value (recommended 0.05–0.10).
   - Set `global_velocity_feedback` to 0.0 (or near-zero) so global volume projection remains positional only.
3. Keep adaptive substepping, local volume correction, implicit MCF, collision, and global volume correction ordering unchanged to preserve behavior and maintain minimal diff.

3. **Phase 3: Make global correction less aggressive (depends on Phase 2)**
1. In `apply_global_volume_correction(...)`, reduce per-iteration displacement cap:
   - Change `d_max` from `0.15 * mean_edge_len` to `0.06–0.08 * mean_edge_len`.
2. Keep iterative structure and fixed-mask behavior unchanged; only tune correction magnitude to avoid over-shoot collapse.

4. **Phase 4: Paper-faithfulness consistency pass (parallel with Phase 3 after edits identified)**
1. Keep implicit MCF solve form as currently implemented (`M - gamma*dt*L` with current Laplacian sign convention) and retain CG failure fallback.
2. Avoid adding ad-hoc extra damping/clamps beyond the single final speed clamp; this aligns with user request to remove redundant stabilizers.

5. **Phase 5: Verification (depends on Phases 2–4)**
1. Run static diagnostics (`get_errors`) on `python_sim_dev/water_sim_basic_clean_based.py`.
2. Run short simulation smoke test in existing notebook loop (`python_sim_dev/sim.ipynb`) without changing notebook parameters:
   - Verify no NaN/Inf fallback triggers.
   - Confirm droplet no longer exhibits rapid over-expansion followed by violent collapse.
   - Confirm mesh quality ratio remains bounded (no immediate edge blow-up) over initial 200–300 steps.
3. Compare qualitative behavior against current `water_sim_basic.py` baseline: slower drift to stable shape, reduced oscillation amplitude.

**Relevant files**
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/python_sim_dev/water_sim_basic_clean_based.py` — primary edit target (`step`, `apply_global_volume_correction`), remove redundant clamp and reduce correction feedback.
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/python_sim_dev/sim.ipynb` — verification-only runtime check (no planned edits).
- `/Users/meli/Desktop/Kevin/UCB/CS 184/CS184-final-project/python_sim_dev/water_sim_basic.py` — reference behavior/pattern for adaptive stepping and conservative stabilization.

**Verification**
1. `get_errors` on `python_sim_dev/water_sim_basic_clean_based.py`.
2. Execute existing `sim.ipynb` run cell with unchanged `dt` and inspect first 200–300 frames.
3. Check that final speed clamp is the only clamp in `step(...)` and feedback constants match chosen conservative values.

**Decisions**
- Include: minimal code-only edits in `python_sim_dev/water_sim_basic_clean_based.py`.
- Exclude: notebook parameter tuning changes.
- Include: retain current architecture and ordering; only simplify/tune conflicting stabilizers.

**Further Considerations**
1. Conservative feedback setting recommendation: Option A (safer) `mcf_velocity_feedback=0.05`, `global_velocity_feedback=0.0`; Option B (slightly more responsive) `0.10` and `0.0`.
2. Global correction cap recommendation: start at `d_max = 0.06 * mean_edge_len`; if convergence is too slow, increase to `0.08`.
