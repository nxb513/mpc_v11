function bench_job(j, out_dir, n_seeds)
% BENCH_JOB Runs the benchmark cases assigned to CI job j (round-robin:
% cases j, j + n_jobs, j + 2 n_jobs, ...), each with bench_case.m.
if nargin < 2 || isempty(out_dir), out_dir = 'results'; end
if nargin < 3, n_seeds = []; end
if ischar(j) || isstring(j), j = str2double(j); end
cfg = experiment_config();
ids = j:cfg.n_jobs:cfg.n_cases;
fprintf('Job %d of %d: cases %s\n', j, cfg.n_jobs, mat2str(ids));
for id = ids
    bench_case(id, out_dir, n_seeds);
end
end
