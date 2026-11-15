function st = paper_palette()
% PAPER_PALETTE Colour-blind-safe colours (Okabe-Ito palette) combined with
% distinct line styles and markers, so that every series is also identifiable in
% a greyscale print. Order: Fixed N = 5, Fixed N = 10, nominal-model scheduler,
% proposed Tiny-ML scheduler.
st.color = [0.000 0.447 0.698;    % blue
            0.835 0.369 0.000;    % vermillion
            0.800 0.475 0.655;    % reddish purple
            0.000 0.620 0.451];   % bluish green
st.line   = {'-', '--', '-.', '-'};
st.marker = {'o', 's', 'd', '^'};
st.grey   = [0.75 0.75 0.75];
% Diverging map for relative changes: blue = improvement, white = 0,
% vermillion = deterioration (avoids the red-green pair).
n = 128;
neg = [linspace(st.color(1, 1), 1, n)', linspace(st.color(1, 2), 1, n)', linspace(st.color(1, 3), 1, n)'];
pos = [linspace(1, st.color(2, 1), n)', linspace(1, st.color(2, 2), n)', linspace(1, st.color(2, 3), n)'];
st.diverging = [neg; pos];
end
