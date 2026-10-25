function rows = run_benchmark(methods, caseNumbers)
% Full fresh experiment by default. Failures are recorded, never cached away.
if nargin < 1, methods = {'BRS','HA','BIAGT','GHA','CC-A*'}; end
if nargin < 2, caseNumbers = 1:100; end
if ischar(methods) || isstring(methods), methods = cellstr(methods); end
root = miningpocket_setup(); rows = struct([]);
for m = 1:numel(methods)
    for i = caseNumbers
        row = struct('id',sprintf('B%03d',i),'method',char(methods{m}),...
            'success',false,'J',NaN,'totalTime',NaN,'extraStops',NaN,...
            'totalPlanningSeconds',NaN,'error','');
        try
            r = plan_case(i,methods{m}); row.success = r.success;
            row.totalPlanningSeconds = r.totalPlanningSeconds; row.J = r.J;
            if ~isempty(r.execution)
                row.totalTime = r.execution.totalTime;
                row.extraStops = r.execution.internalStops;
            end
        catch err
            row.error = err.message;
        end
        if isempty(rows), rows = row; else, rows(end+1) = row; end
        evalcw.writeJson(fullfile(root,'results','reproduced','fresh_results.json'),rows);
    end
end
end
