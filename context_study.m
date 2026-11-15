function res = context_study()
% CONTEXT_STUDY Trains the fixed Tiny-ML architecture (selected once by
% architecture_search.m on the nominal context) on several hidden latency
% formulas and reports the amount of data needed for each.
%
% Protocol (one run per latency context = one set of hidden formula parameters):
%   1. The latency formula (true_latency.m, parameters in latency_context.m) is
%      hidden from the learner; it is only used to generate closed-loop data.
%   2. A dataset is generated for the context (new seeds), split by rollout into
%      a training pool (70 %), validation (15 %) and test (15 %).
%   3. The same architecture is trained from scratch on n samples of the pool,
%      n = 25 ... n_max, with R repeats (different subsets / initialisations).
%   4. Learning curve = test MAE vs n. Saturation n_sat = smallest n whose mean
%      test MAE is within SAT_TOL of the value at n_max, provided the last
%      doubling (n_max/2 -> n_max) improved the MAE by less than SAT_TOL;
%      otherwise the curve has not settled and n_sat is reported as NaN.
% n_sat is the number of logged solves to collect on a new platform.
%
% Exports Fig_7.pdf (learning curves of all contexts, relative MAE, saturation
% markers) and prints the saturation table.

% Runs every context (context_case.m), then combines and plots. On CI the
% contexts run as parallel jobs and combine_context_results.m is called instead.

ctx = context_list();
fprintf('\n--- Tiny-ML (fixed architecture) on hidden latency formulas ---\n');
for k = 1:numel(ctx)
    context_case(k);
end
res = combine_context_results('results');
end
