function export_figure(fh, base_name)
% EXPORT_FIGURE Writes <base_name>.pdf and <base_name>.eps as vector graphics
% with embedded fonts.
try
    exportgraphics(fh, [base_name, '.pdf'], 'ContentType', 'vector');
    exportgraphics(fh, [base_name, '.eps'], 'ContentType', 'vector');
catch
    print(fh, [base_name, '.pdf'], '-dpdf', '-vector');
    print(fh, [base_name, '.eps'], '-depsc', '-vector');
end
end
