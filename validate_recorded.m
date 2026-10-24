function rows = validate_recorded(caseNumbers)
% Independently audit geometry and time constraints in the supplied results.
if nargin < 1, caseNumbers = 1:100; end
root = miningpocket_setup(true); methods = {'Ours','HA','BIAGT','GHA'};
settings = jsondecode(fileread(fullfile(root,'experiment_settings.json')));
rows = struct([]);
for m = 1:numel(methods)
    for i = caseNumbers
        id = sprintf('B%03d',i);
        d = load(fullfile(root,'benchmark','cases',[id '.mat']),'scene');
        raw = load(fullfile(root,'results','recorded',[methods{m} '_' id '.mat']));
        r = raw.r;
        assert(r.success,'Stored planner reported failure.');
        a = evalcw.independent(d.scene,r.best);
        t = evalcw.profile(evalcw.pieces(r.best),d.scene.truck,evalcw.options());
        row = struct('id',id,'method',methods{m},'passed',a.passed && t.valid,...
            'totalTime',t.totalTime,'extraStops',t.internalStops,...
            'J',t.totalTime+settings.lambda_s*t.internalStops,...
            'positionError',a.poseError(1),'headingError',a.poseError(2));
        if isempty(rows), rows = row; else, rows(end+1) = row; end
        fprintf('audit %s %s: %d\n',methods{m},id,rows(end).passed);
    end
end
evalcw.writeJson(fullfile(root,'results','reproduced','independent_audit.json'),rows);
assert(all([rows.passed]),'One or more independent checks failed.');
end
