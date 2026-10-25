function result = plan_case(caseNumber, method)
% Replan from the fixed input. Recorded solutions never enter the planner.
if nargin < 2, method = 'BRS'; end
root = miningpocket_setup();
validateattributes(caseNumber,{'numeric'},{'scalar','integer','>=',1,'<=',100});
method = upper(char(method)); id = sprintf('B%03d',caseNumber);
data = load(fullfile(root,'benchmark','cases',[id '.mat']),'scene');
scene = data.scene;
timer = tic;
switch method
    case {'BRS','OURS'}
        method = 'BRS'; r = cm.solve(scene);
    case 'HA'
        r = base.search(scene,'HA',base.options());
    case {'CC-A*','CCA'}
        method = 'CC-Astar'; r = base.ccsearch(scene,base.options());
    case 'BIAGT'
        r = base.biagt(scene,base.options());
    case {'GHA','GHA-SEARCH'}
        method = 'GHA'; r = base.search(scene,'GHA',base.options());
    otherwise
        error('MiningPocket:Method','Use BRS, HA, BIAGT, GHA, or CC-A*.');
end
planningSeconds = toc(timer);
result = struct('id',id,'method',method,'success',false,'planner',r,...
    'planningSeconds',planningSeconds,'profileSeconds',0,'totalPlanningSeconds',planningSeconds,...
    'execution',[],'validation',[],'J',NaN);
if r.success
    timer = tic;
    result.execution = evalcw.profile(evalcw.pieces(r.best),scene.truck,evalcw.options());
    result.profileSeconds = toc(timer);
    result.totalPlanningSeconds = planningSeconds + result.profileSeconds;
    result.validation = evalcw.independent(scene,r.best);
    result.success = result.execution.valid && result.validation.passed;
    settings = jsondecode(fileread(fullfile(root,'experiment_settings.json')));
    result.J = result.execution.totalTime + settings.lambda_s * result.execution.internalStops;
end
save(fullfile(root,'results','reproduced',[method '_' id '.mat']),'result','-v7');
fprintf('%s %s: valid=%d, planning=%.3f s, J=%.6f s\n',...
    method,id,result.success,result.totalPlanningSeconds,result.J);
end
