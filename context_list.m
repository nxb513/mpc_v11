function ctx = context_list()
% CONTEXT_LIST Hidden latency formulas (latency_context.m parameters) and flight
% settings of the Tiny-ML study (context_case.m).
ctx = {
  struct('label', 'C0 nominal',              'lat', struct('name', 'C0'), 'fl', struct())
  struct('label', 'C1 faster SoC (x1.6)',    'lat', struct('name', 'C1', 'R_scale', 1.6), 'fl', struct())
  struct('label', 'C2 slower SoC (x0.6)',    'lat', struct('name', 'C2', 'R_scale', 0.6), 'fl', struct())
  struct('label', 'C3 costlier iteration',   'lat', struct('name', 'C3', 'c_iter', 10), 'fl', struct())
  struct('label', 'C4 OS overhead +0.8 ms',  'lat', struct('name', 'C4', 'overhead_ms', 0.8), 'fl', struct())
  struct('label', 'C5 other contention law', 'lat', struct('name', 'C5', 'contention', 'power', 'cont_a', 4, 'cont_b', 2), 'fl', struct())
  struct('label', 'C6 timing jitter 15%',    'lat', struct('name', 'C6', 'jitter_cv', 0.15), 'fl', struct())
  struct('label', 'C7 aggressive + combined','lat', struct('name', 'C7', 'R_scale', 0.8, 'overhead_ms', 0.4, ...
        'contention', 'power', 'cont_a', 6, 'cont_b', 3, 'jitter_cv', 0.10), ...
        'fl', struct('dist_scale', 2.0, 'init_err', 1.0, 'traj_set', {{'racing'}}))
};
end
