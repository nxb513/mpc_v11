# Computation-Aware Adaptive LTV-MPC for Quadrotors near the Actuator Limits

MATLAB simulation code for a computation-aware adaptive model predictive controller whose horizon is scheduled by a Tiny-ML latency predictor.

## Problem setting

| Item | Choice | Source |
|---|---|---|
| Plant | Nonlinear 6-DOF rigid body, Z-Y-X Euler angles, RK4 (`quad_nl_dynamics.m`, `quad_nl_step.m`) | [S2] eqs. 1–4, 16 |
| Platform | Standard Quad: m = 1 kg, l = 0.15 m, J = diag(5, 5, 10) g·m², 0.25 ≤ T_i ≤ 5 N, c_τ = 0.01, ω_max = 10 rad/s (`quad_params.m`) | [S2] Table I |
| Controller | LTV-MPC linearised along the reference (`ltv_model.m`, `build_condensed_qp_ltv.m`). Per-rotor thrust limits are hard; the body-rate limit is soft | |
| Weights | Bryson's rule (`mpc_weights.m`). Input maxima follow from the actuator limits. State tolerances have two levels: "tight" (default) and "loose" (Part C) | Bryson & Ho; Franklin, Powell & Emami-Naeini |
| Terminal cost | DARE of the hover model | |
| References | Flatness-based figure-8 (lemniscate), wave-circle (helix) and racing circuit, sized for a 10 × 10 × 5 m arena (`generate_trajectories.m`) | |
| Scenario ladder | S1 nominal, S2 half-way, S3 at the rotor limits, then feed-forward rotor thrust beyond the limits 10 % (S4), 20 % (S5) and 30 % (S6) of the time. Levels are found by bisection (`scenario_levels.m`) | Actuator limits of [S2] |
| Computer | ARM Cortex-A7 timing model (`latency_calibration.m`, `true_latency.m`). The computation budget is one sampling period | [S1] Table 2 |
| Experiment axes | Control rate 25/50/75/100 Hz; mean background CPU load 20/50/80 %; acceleration disturbance std 0/0.5/1 m/s² (`simulate_workload.m`) | |
| Deadline miss | The next input of the last successful plan is applied (shifted plan). If that plan is exhausted, hover LQR is used | |

[S1] J. Frey, S. Di Cairano, R. Quirynen, "Active-Set based Inexact Interior Point QP Solver for Model Predictive Control", IFAC World Congress 2020 (MERL TR2020-105).
[S2] P. Foehn, A. Romero, D. Scaramuzza, "Time-Optimal Planning for Quadrotor Waypoint Flight", *Science Robotics* 6(56), 2021.

## Controllers (`adaptive_mpc_solver.m`)

| Controller | Horizon choice at every step |
|---|---|
| Fixed N = 5 | always 5 |
| Fixed N = 10 | always 10 |
| Nominal-model scheduler | the largest (N, I_max) whose latency from the nominal formula (`qp_latency_model.m`, worst-case iterations) fits the budget |
| Proposed (Tiny-ML scheduler) | the largest (N, I_max) whose latency upper bound from the Tiny-ML quantile model fits the budget |

* Candidate horizons are N ∈ {4, 6, 8, 10, 12, 16}.
* Candidate iteration caps are I_max ∈ {10, 20, 40}.
* All controllers share the same weights, constraints and fallback.

## Tiny-ML latency predictor

* **Black-box protocol:** `true_latency.m` is used only to generate data. The learner receives `(X, y)` pairs with `X = [N, I_max, CPU load, ||e_pos||, ||e_vel||]`.
* **Data:** closed-loop rollouts over all scenario levels, control rates, loads and disturbances (`generate_large_dataset.m`). The split into train, validation and test sets is made by rollout (`split_by_group.m`).
* **Architecture (`architecture_search.m`):** the smallest network whose validation MAE is within 5 % of the best candidate. It is drawn in `Fig_6.pdf`.
* **Upper-bound model (`train_model.m`):** same architecture, pinball loss at the 0.95 quantile, and a split-conformal factor computed on the validation rollouts.
* **Learning curves:**
  * nominal formula: `Fig_4.pdf` (training-set size) and `Fig_5.pdf` (epochs);
  * eight hidden latency formulas, each retrained from scratch: `Fig_7.pdf` (`context_study.m`, `context_case.m`).

## Experiments (`experiment_config.m`)

| Part | Grid | Cases |
|---|---|---|
| A | 3 trajectories × 6 levels × 4 control rates × 3 mean CPU loads (disturbance 0.5 m/s²) | 216 |
| B | 3 trajectories × 6 levels × disturbance {0, 1.0} m/s² (50 Hz, CPU 50 %) | 36 |
| C | 3 trajectories × 6 levels with the loose weights (50 Hz, CPU 50 %, disturbance 0.5 m/s²) | 18 |

Every case is run on 30 disturbance and CPU-load realisations (seeds). All controllers use the same realisations, so comparisons between controllers are paired.

**Metrics** (`report_results.m`):
* position RMSE: mean and 95 % bootstrap CI over the non-crashed seeds;
* crashes: position error above 5 m;
* deadline-miss rate;
* solver time per second of flight;
* mean selected N;
* relative RMSE difference between Proposed and each baseline on the seeds where neither crashed, with a 95 % bootstrap CI.

**Figures:**
| Figure | Content |
|---|---|
| `Fig_1` | scheduler trace |
| `Fig_2` | RMSE vs scenario level |
| `Fig_3` | crashes, deadline misses and solver time |
| `Fig_4`–`Fig_7` | Tiny-ML |
| `Fig_8` | relative RMSE change over control rate × CPU load |
| `Fig_9` | sensitivity to disturbance and weights |

## Limitations

* Latencies come from a timing model calibrated on published data, not from on-board measurement.
* [S2] gives no drag coefficient for the Standard Quad, so the disturbance is modelled as an acceleration rather than a wind velocity.
* Iteration counts of MATLAB `quadprog` (active-set) stand in for those of an embedded active-set solver.

## Running

The code requires MATLAB (tested with R2024a), the Control System Toolbox and the Optimization Toolbox.

* **Sequential run:** `main_simulation`.
* **Parallel run:** `.github/workflows/matlab_ci.yml` runs on every push to `main`. It executes the same steps on GitHub Actions:
  1. one training job;
  2. 60 benchmark jobs (`bench_job.m`) and 8 latency-context jobs (`context_case.m`);
  3. one report job, which builds the figures and tables.

Results of the released version are summarised in `RESULTS.md`.
