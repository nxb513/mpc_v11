# Computation-Aware Adaptive LTV-MPC for Quadrotors near the Actuator Limits

MATLAB simulation code for the paper on computation-aware adaptive MPC with a Tiny-ML latency predictor.

## Problem setting

| Item | Choice | Source / rationale |
|---|---|---|
| Plant | Nonlinear 6-DOF rigid body, Z-Y-X Euler angles, RK4 (`quad_nl_dynamics.m`, `quad_nl_step.m`) | [S2] eqs. 1–4, 16 |
| Platform | **Standard Quad**: m = 1 kg, l = 0.15 m, J = diag(5, 5, 10) g·m², 0.25 ≤ T_i ≤ 5 N, c_τ = 0.01, ω_max = 10 rad/s (`quad_params.m`) | [S2] Table I. Its thrust-to-weight ratio of about 2 means that aggressive references can demand more than the rotors can give |
| Controller | LTV-MPC linearised along the reference (`ltv_model.m`, `build_condensed_qp_ltv.m`). Per-rotor thrust limits are hard; the body-rate limit is soft | |
| Weights | Bryson's rule (`mpc_weights.m`): input maxima come from the actuator limits. Acceptable state deviations are set at two levels, "tight" (default) and "loose" (Part 4) | Bryson & Ho; Franklin, Powell & Emami-Naeini |
| Terminal cost | DARE of the hover model | |
| References | Flatness-based figure-8, wave-circle and racing circuit, sized for a 10 × 10 × 5 m arena (`generate_trajectories.m`) | |
| Scenario ladder | S1 nominal, S2 half-way, S3 at the rotor limits, then beyond the limits 10 % (S4), 20 % (S5) and 30 % (S6) of the time. Levels are found by bisection on the feed-forward rotor thrust (`scenario_levels.m`) | Physical actuator limits only |
| Computer | ARM Cortex-A7 timing model (`latency_calibration.m`, `true_latency.m`). The budget is one sampling period | [S1] Table 2 |
| Control rate, background CPU load, disturbance | Experiment axes: 25/50/75/100 Hz, mean load 20/50/80 %, acceleration disturbance std 0/0.5/1 m/s² (`simulate_workload.m`) | Swept, not claimed |
| Deadline miss | Apply the next input of the last successful plan (shifted plan). If that plan is exhausted, use hover LQR | Standard practice in real-time MPC |

[S1] J. Frey, S. Di Cairano, R. Quirynen, IFAC World Congress 2020 (MERL TR2020-105).
[S2] P. Foehn, A. Romero, D. Scaramuzza, *Science Robotics* 6(56), 2021.

## Controllers compared (`adaptive_mpc_solver.m`)

| Controller | Horizon choice at every step |
|---|---|
| Fixed N = 5 | always 5 |
| Fixed N = 10 | always 10 |
| Nominal-model scheduler | the largest (N, I_max) whose latency from the **nominal formula** (calibration known before flight, worst-case iterations) fits the budget |
| **Proposed (Tiny-ML scheduler)** | the largest (N, I_max) whose latency **upper bound predicted by the Tiny-ML quantile model** (learned from logged data, formula unknown, conformal-calibrated to 95 % coverage) fits the budget |

Candidate horizons are N ∈ {4, 6, 8, 10, 12, 16} and iteration caps I_max ∈ {10, 20, 40}.
All controllers share the same weights, constraints and fallback.

## Tiny-ML latency predictor

* **Hidden formula:** `true_latency.m` is used **only** to generate data. The learner sees only `(X, y)` with `X = [N, I_max, CPU load, ||e_pos||, ||e_vel||]`.
* **Data:** closed-loop rollouts over all scenario levels, rates, loads and disturbances. The data is split by rollout (`split_by_group.m`).
* **Architecture selection (`architecture_search.m`):** pick the smallest network whose validation MAE is within 5 % of the best. The selected network is drawn in `Fig_6.pdf`.
* **Upper-bound predictor (`train_model.m`):** the same architecture is trained with the pinball loss at the 0.95 quantile. A split-conformal factor from the validation rollouts then gives 95 % coverage on held-out data. The test coverage is reported.
* **Learning curves:**
  * nominal formula: `Fig_4.pdf` (samples) and `Fig_5.pdf` (epochs);
  * 8 hidden latency formulas, each retrained from scratch: `Fig_7.pdf` (`context_study.m`). The saturation point tells how many samples to collect before deployment.

## Experiments (`experiment_config.m`, `bench_job.m`, `report_results.m`)

| Part | Grid | Cases |
|---|---|---|
| A | 3 trajectories × 6 levels × 4 control rates × 3 mean CPU loads (disturbance 0.5 m/s²) | 216 |
| B | 3 trajectories × 6 levels × disturbance {0, 1.0} m/s² (50 Hz, CPU 50 %) | 36 |
| C | 3 trajectories × 6 levels with the loose Bryson weights | 18 |
| Tiny-ML | 8 hidden latency formulas, learning curves (`context_case.m`) | 8 |

Figures: `Fig_1` scheduler trace, `Fig_2` RMSE vs level, `Fig_3` crashes / misses / CPU time, `Fig_4`–`Fig_7` Tiny-ML, `Fig_8` improvement over rate × CPU load, and `Fig_9` robustness to disturbance and weights.

Every case is run on **30 independent disturbance and CPU-load realisations (seeds)**. All controllers use the same realisations, so the comparisons are paired.

Metrics: position RMSE (mean with a 95 % bootstrap CI over the non-crashed seeds), crashes (position error above 5 m), deadline-miss rate, solver time per second of flight, and mean selected N. Proposed is compared with each baseline on the same seeds: mean difference with a 95 % bootstrap CI, and the share of seeds won.

## Known limitations

* Latencies come from a calibrated timing model, not from on-board measurement. The paper must say so.
* The Standard Quad has no drag coefficient in [S2], so the disturbance is an acceleration, not a wind-velocity model.
* MATLAB `quadprog` iteration counts are used as a proxy for an embedded active-set solver.

## How to run

Requirements: MATLAB R2021b or newer, with the Control System Toolbox and the Optimization Toolbox.

```matlab
main_simulation
```
