# Computation-Aware Adaptive LTV-MPC for Quadrotors near the Actuator Limits

MATLAB simulation code for the paper on computation-aware adaptive MPC with a Tiny-ML latency predictor.
Results of earlier versions are archived in `_rac/` (not tracked by git) and must not be reused.

## Problem setting

| Item | Choice | Source / rationale |
|---|---|---|
| Plant | Nonlinear 6-DOF rigid body, Z-Y-X Euler angles, RK4 (`quad_nl_dynamics.m`, `quad_nl_step.m`) | [S2] eqs. 1–4, 16 |
| Platform | **Standard Quad**: m = 1 kg, l = 0.15 m, J = diag(5, 5, 10) g·m², 0.25 ≤ T_i ≤ 5 N, c_τ = 0.01, ω_max = 10 rad/s (`quad_params.m`) | [S2] Table I. Its thrust-to-weight ratio of about 2 means that aggressive references can demand more than the rotors can give |
| Controller | LTV-MPC linearised along the reference (`ltv_model.m`, `build_condensed_qp_ltv.m`). Per-rotor thrust limits are hard; the body-rate limit is soft | |
| Weights | Bryson's rule (`mpc_weights.m`): input maxima come from the actuator limits. Acceptable state deviations are set at two levels, "tight" (default) and "loose" (Part 4) | Bryson & Ho; Franklin, Powell & Emami-Naeini |
| Terminal cost | DARE of the hover model | |
| References | Flatness-based figure-8, wave-circle and racing circuit, sized for a 10 × 10 × 5 m arena (`generate_trajectories.m`) | |
| Scenario ladder | S1 nominal, S2 at the rotor limits, S3 beyond the limits 10 % of the time, S4 beyond the limits 30 % of the time. Levels are found by bisection on the feed-forward rotor thrust (`scenario_levels.m`) | Physical actuator limits only |
| Computer | ARM Cortex-A7 timing model (`latency_calibration.m`, `true_latency.m`). The budget is one sampling period | [S1] Table 2 |
| Background CPU load, disturbance | Experiment axes: mean load 20/50/80 %, acceleration disturbance std 0/0.5/1 m/s² (`simulate_workload.m`) | Swept, not claimed |
| Deadline miss | Apply the next input of the last successful plan (shifted plan). If that plan is exhausted, use hover LQR | Standard practice in real-time MPC |

[S1] J. Frey, S. Di Cairano, R. Quirynen, IFAC World Congress 2020 (MERL TR2020-105).
[S2] P. Foehn, A. Romero, D. Scaramuzza, *Science Robotics* 6(56), 2021.

## Controllers compared (`adaptive_mpc_solver.m`)

| Controller | Horizon choice at every step |
|---|---|
| Fixed N = 5 | always 5 |
| Fixed N = 10 | always 10 |
| Nominal-model scheduler | the largest (N, I_max) whose latency from the **nominal formula** (calibration known before flight, worst-case iterations) fits the budget |
| **Proposed (Tiny-ML scheduler)** | the largest (N, I_max) whose latency **predicted by the Tiny-ML model** (learned from logged data, formula unknown), times its data-driven safety factor, fits the budget |

Candidate horizons are N ∈ {4, 6, 8, 10, 12, 16} and iteration caps I_max ∈ {10, 20, 40}.
All controllers share the same weights, constraints and fallback.

## Tiny-ML latency predictor

* **Hidden formula:** `true_latency.m` is used **only** to generate data. The learner sees only `(X, y)` with `X = [N, I_max, CPU load, ||e_pos||, ||e_vel||]`.
* **Data:** closed-loop rollouts over all scenario levels, rates, loads and disturbances. The data is split by rollout (`split_by_group.m`).
* **Architecture selection (`architecture_search.m`):** pick the smallest network whose validation MAE is within 5 % of the best. The selected network is drawn in `Fig_6.pdf`.
* **Safety factor:** the 95th percentile of true/predicted latency on the validation rollouts (`train_model.m`).
* **Learning curves:**
  * nominal formula: `Fig_4.pdf` (samples) and `Fig_5.pdf` (epochs);
  * 8 hidden latency formulas, each retrained from scratch: `Fig_7.pdf` (`context_study.m`). The saturation point tells how many samples to collect before deployment.

## Experiments (`main_simulation.m`)

1. **Scenario ladder** S1–S4 × 3 trajectories at 50 Hz (`Fig_1.pdf`, `Fig_2.pdf`, `Fig_3.pdf`).
2. **Control-rate sweep** from 20 to 100 Hz at S3 (`Fig_8.pdf`).
3. **Mean CPU load × disturbance grid** at S3 (`Fig_9.pdf`).
4. **Loose Bryson weights** at S3 (sensitivity check).
5. **Tiny-ML study** on 8 hidden latency formulas (`Fig_7.pdf`).

Every case is run on **20 independent disturbance and CPU-load realisations (seeds)**. All controllers use the same realisations, so the comparisons are paired.

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
