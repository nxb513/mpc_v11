function [i_tr, i_va, i_te] = split_by_group(G, fr_tr, fr_va, seed)
% SPLIT_BY_GROUP Train/validation/test split by rollout id: all samples of one
% closed-loop rollout go to the same subset, so correlated consecutive samples
% cannot leak from training into validation/test.
rng(seed);
g = unique(G);
g = g(randperm(numel(g)));
n_tr = round(fr_tr * numel(g));
n_va = round(fr_va * numel(g));
i_tr = find(ismember(G, g(1:n_tr)));
i_va = find(ismember(G, g(n_tr+1:n_tr+n_va)));
i_te = find(ismember(G, g(n_tr+n_va+1:end)));
end
