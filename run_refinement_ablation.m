function rows = run_refinement_ablation(caseNumbers)
% Same coarse graph, with or without incumbent-preserving local refinement.
if nargin < 1, caseNumbers = 1:100; end
root = miningpocket_setup(); rows = struct([]);
for i = caseNumbers
    id = sprintf('B%03d',i);
    d = load(fullfile(root,'benchmark','cases',[id '.mat']),'scene');
    oc = dp.options(); oc.refine = false;
    coarse = dp.plan(d.scene,oc); fine = dp.plan(d.scene,dp.options());
    row = struct('id',id,'coarseSuccess',coarse.success,'fineSuccess',fine.success,...
        'coarseLength',NaN,'fineLength',NaN,'coarseSeconds',coarse.seconds,'fineSeconds',fine.seconds);
    if coarse.success, row.coarseLength = coarse.best.length; end
    if fine.success, row.fineLength = fine.best.length; end
    if coarse.success, assert(fine.success && fine.best.length <= coarse.best.length+1e-7); end
    if isempty(rows), rows = row; else, rows(end+1) = row; end
    save(fullfile(root,'results','reproduced',['ablation_' id '.mat']),'coarse','fine','-v7');
end
evalcw.writeJson(fullfile(root,'results','reproduced','refinement_ablation.json'),rows);
end
