%% Computation-Aware Adaptive LTV-MPC for Quadrotors near the Actuator Limits
% Sequential local run of the whole study. On GitHub Actions the same steps run
% as parallel jobs (.github/workflows/matlab_ci.yml):
%   1. train      : dataset -> architecture search -> Tiny-ML -> learning curves
%   2. bench      : one job per case of experiment_config.m   (bench_case.m)
%   3. contexts   : one job per hidden latency formula         (context_case.m)
%   4. report     : tables and figures                         (report_results.m,
%                                                               combine_context_results.m)
clear; clc; close all;
fprintf('=================================================================\n');
fprintf(' Computation-Aware Adaptive LTV-MPC Benchmark Suite\n');
fprintf('=================================================================\n');

% 1. Tiny-ML latency predictor (hidden latency formula -> data -> train)
clear tiny_ml_predictor;
generate_large_dataset();
architecture_search();
train_model();
learning_curve();           % Fig_4.pdf, Fig_5.pdf
plot_architecture();        % Fig_6.pdf

% 2. Closed-loop benchmark cases
cfg = experiment_config();
for id = 1:cfg.n_cases
    bench_case(id);
end

% 3. Tiny-ML on hidden latency formulas
ctx = context_list();
for k = 1:numel(ctx)
    context_case(k);
end

% 4. Report
report_results('results');            % Fig_1, 2, 3, 8, 9
combine_context_results('results');   % Fig_7
fprintf('\nSimulation complete. Exported Fig_1.pdf to Fig_9.pdf (Vector PDF).\n');
