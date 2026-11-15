# Final results (paper version `v1.0-paper`)

* **Code:** commit `bd2ca4e`, tagged `v1.0-paper`.
* **Run:** GitHub Actions run 43718863143, 15 Nov 2026, 13:24–14:20 UTC, 70 / 70 jobs successful. MATLAB R2024a, `ubuntu-latest`.
* **Raw outputs:** attached to the `v1.0-paper` release (figures, logs, `.mat` data).

The full setup is described in `README.md`.

## 1. Tiny-ML latency predictor (hidden latency formula)

| Item | Value |
|---|---|
| Data | 24,000 closed-loop samples, 87 rollouts. Train / validation / test = 17,505 / 2,577 / 3,918 samples (61 / 13 / 13 rollouts) |
| Selected architecture (`architecture_search.m`) | 5–64–32–1 ReLU, linear target, 2,497 parameters, 4,800 FLOPs per inference |
| Test accuracy | MAE 7.17 ms, RMSE 19.48 ms, R² = 0.969 (linear regression: MAE 50.46 ms, R² = 0.530) |
| Upper-bound model (q = 0.95, pinball loss + split-conformal) | Conformal factor 0.993; test coverage 90.5 % (target 95 %); mean over-estimate 38.3 % |
| Learning curve (nominal) | Saturation (within 5 % of the full-data MAE) at n = 12,800 samples. Best epoch 92 (early stop at 132) |

### Eight hidden latency formulas (same architecture, retrained from scratch, 5 repeats)

| Context | Mean latency | n_sat | Tiny-ML rel. MAE | Linear rel. MAE | R² |
|---|---|---|---|---|---|
| C0 nominal | 72.5 ms | 12,800 | 10.9 % | 76.3 % | 0.961 |
| C1 faster SoC (×1.6) | 50.1 ms | 12,800 | 11.6 % | 69.8 % | 0.961 |
| C2 slower SoC (×0.6) | 150.2 ms | 12,800 | 8.1 % | 71.1 % | 0.961 |
| C3 costlier iteration | 81.7 ms | 12,800 | 12.2 % | 78.7 % | 0.945 |
| C4 OS overhead +0.8 ms | 81.9 ms | 12,800 | 8.9 % | 76.0 % | 0.930 |
| C5 other contention law | 62.0 ms | 6,400 | 10.6 % | 42.0 % | 0.941 |
| C6 timing jitter 15 % | 81.5 ms | 6,400 | 16.8 % | 79.4 % | 0.940 |
| C7 aggressive + combined | 85.3 ms | 12,800 | 12.6 % | 52.9 % | 0.954 |

## 2. Closed-loop benchmark (270 cases × 30 paired seeds = 8,100 flights per controller)

### Crashes (position error > 5 m)

| Controller | Crashed runs |
|---|---|
| Fixed-MPC (N = 5) | 1,796 (22.2 %) |
| Fixed-MPC (N = 10) | 3,953 (48.8 %) |
| Nominal-model scheduler | 953 (11.8 %) |
| **Proposed (Tiny-ML scheduler)** | **980 (12.1 %)** |

### Paired comparison

A case is counted when the 95 % bootstrap CI of the relative RMSE difference excludes 0.

| Proposed vs | Better | Worse | Not significant | Not comparable (crashes) |
|---|---|---|---|---|
| Fixed-MPC (N = 5) | **159** | 52 | 15 | 44 |
| Nominal-model scheduler | **125** | 62 | 62 | 21 |
| Fixed-MPC (N = 10) | 35 | 107 | 10 | 118 |

The Fixed N = 10 comparison is biased by survivorship: it crashes in 48.8 % of the flights, and its surviving runs are the easy ones.

### Part A by scenario level

Part A pools all trajectories, rates and CPU loads. RMSE is the mean over non-crashed runs.

| Level | Fixed N=5 | Fixed N=10 | Nominal scheduler | Proposed | Proposed better / worse (of 36) vs Fixed N=5 | vs Nominal |
|---|---|---|---|---|---|---|
| S1 nominal | 0.0808 m, 0.0 % crash | 0.0800, 0.0 % | 0.0810, 0.0 % | 0.0809, 0.0 % | 23 / 13 | 28 / 0 |
| S2 half-way | 0.0937, 0.1 % | 0.0747, 0.0 % | 0.0965, 0.1 % | 0.0959, 0.1 % | 21 / 15 | 29 / 0 |
| S3 at limit | 0.0994, 26.7 % | 0.0686, 40.8 % | 0.1155, 16.6 % | 0.1128, 16.9 % | 21 / 6 | 21 / 2 |
| S4 beyond 10 % | 0.1057, 33.5 % | 0.0893, 65.3 % | 0.1172, 19.1 % | 0.1136, 19.4 % | 20 / 4 | 12 / 10 |
| S5 beyond 20 % | 0.1162, 45.1 % | 0.1150, 90.1 % | 0.1172, 23.9 % | 0.1236, 23.9 % | 15 / 2 | 7 / 12 |
| S6 beyond 30 % | 0.1371, 53.4 % | 0.0903, 98.1 % | 0.1527, 27.9 % | 0.1586, 28.8 % | 13 / 5 | 3 / 18 |

### Proposed vs Fixed N = 5 by condition (Part A)

| Condition | Better / worse / n.s. / not comparable | Mean RMSE change |
|---|---|---|
| CPU 20 % | 64 / 3 / 4 / 1 | −8.4 % |
| CPU 50 % | 32 / 21 / 6 / 13 | −0.7 % |
| CPU 80 % | 17 / 21 / 4 / 30 | +7.9 % |
| 25 Hz | 46 / 4 / 3 / 1 | −4.5 % |
| 50 Hz | 36 / 5 / 3 / 10 | −9.3 % |
| 75 Hz | 16 / 20 / 4 / 14 | +2.2 % |
| 100 Hz | 15 / 16 / 4 / 19 | +7.0 % |
| lemniscate | 31 / 14 / 6 / 21 | −2.2 % |
| helix | 42 / 11 / 3 / 16 | −5.3 % |
| racing | 40 / 20 / 5 / 7 | +1.5 % |
| Part B (disturbance 0 / 1.0) | 28 / 7 / 1 / 0 | |
| Part C (loose weights) | 18 / 0 / 0 / 0 | |
| **All 270 cases** | **159 / 52 / 15 / 44** | **−2.5 %** |

## 3. Conclusions supported by the data

1. **Where computation is available** (CPU load ≤ 50 %, rate ≤ 50 Hz), the Tiny-ML scheduler tracks better than a fixed horizon (Part A with CPU ≤ 50 % and rate ≤ 50 Hz: better in 65, worse in 4 of 72 cases, mean −8.4 % RMSE). It also crashes about half as often as Fixed N = 5 (12.1 % vs 22.2 %).
2. **Without knowing the latency formula**, the Tiny-ML scheduler matches the nominal-model scheduler on safety (12.1 % vs 11.8 % crashes). It is better than that scheduler in 125 of 270 cases.
3. **Fixed long horizons are not real-time feasible** on the Cortex-A7 timing model: Fixed N = 10 crashes in 48.8 % of the flights.

## 4. Limitations to state in the paper

1. **Overloaded processor** (CPU 80 % or 75–100 Hz): no benefit over Fixed N = 5 (mean +7–8 % RMSE). The budget is too small for the ~10 % latency-prediction error.
2. **Most aggressive levels** (S5–S6): the nominal-model scheduler, which in simulation knows the exact formula, is better (12 and 18 worse cases out of 36).
3. **Upper-bound coverage:** 90.5 % on the test set, below the 95 % target.
4. **Simulation only:** latency comes from a timing model calibrated on published Cortex-A7 data, not from on-board measurement.
