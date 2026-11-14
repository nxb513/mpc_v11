# Computation-Aware Multi-Parameter Adaptive MPC for Agile UAVs

This repository contains the official, reproducible MATLAB simulation and benchmarking codebase for the paper **"Computation-Aware Multi-Parameter Adaptive Model Predictive Control for Agile Unmanned Aerial Vehicles"**.

## Key Architectural Highlights

1. **Leak-Free 2-Hidden-Layer Tiny-ML Surrogate (`train_model.m` & `tiny_ml_predictor.m`):**
   * Trained with Huber (Smooth-L1) loss, Adam optimizer, and L2 regularization on hardware-calibrated Active-Set QP profiling samples (`mpc_large_dataset.mat`) across the real-time latency operating regime ($R^2 = 0.9887$, $\text{MAE} = 0.8222\text{ ms}$).
   * Uses strictly raw, independent input features $[N, I^{\max}, \text{CPU}_k, \|e_{\text{pos}}\|, \|e_{\text{vel}}\|, s_{\text{hw}}]$ with zero hand-crafted formula leakage.
   * Performs persistent weight loading (`trained_predictor.mat`) and fast $6 \rightarrow 32 \rightarrow 16 \rightarrow 1$ ReLU forward inference during real-time control.
2. **Active-Set QP & Fair Baseline Benchmarking (`measure_qp_latency.m` & `adaptive_mpc_solver.m`):**
   * Explicitly configures MATLAB's `quadprog` with `'Algorithm', 'active-set'` and valid initial feasible starting point.
   * Evaluates **Anytime MPC** via genuine early termination (returning the unconverged iterate upon budget expiration without arbitrary noise injection).
3. **50-Second Agile Flight Evaluation & Vector PDF Export (`main_simulation.m` & `simulate_workload.m`):**
   * Evaluates closed-loop tracking over an extended $T_{\text{sim}} = 50.0\text{ s}$ horizon ($2{,}001$ control steps at $40\text{ Hz}$) with recurring independent wind gusts and bursty onboard CPU contention.
   * Automatically exports all 4 publication-grade figures as cropped vector PDFs (`Fig_1.pdf` through `Fig_4.pdf`).

---

## How to Run

1. **Prerequisites**: MATLAB (R2021b or newer) with *Control System Toolbox* and *Optimization Toolbox* (zero external statistics/plotting toolbox dependencies).
2. **Run Full Benchmark**:
   ```matlab
   main_simulation
   ```
   *(Automatically generates `Fig_1.pdf` through `Fig_4.pdf` in the working directory).*
