function run_fast100(caseNumbers)
if nargin<1,caseNumbers=1:100;end
root=fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(root); miningpocket_setup(true); audit=fileparts(mfilename('fullpath')); fast=fullfile(audit,'fast_core'); addpath(fast,'-begin'); maxNumCompThreads(1);
out=fullfile(root,'results','reproduced','fast100');if ~isfolder(out),mkdir(out);end
assert(startsWith(which('cc.connect'),fast));disp(which('cc.connect'));rows=struct([]);
for i=caseNumbers
 id=sprintf('B%03d',i);load(fullfile(fullfile(root,'benchmark','cases'),[id '.mat']),'scene');
 r=cm.solve(scene);old=load(fullfile(root,'results','recorded',['Ours_' id '.mat']),'r');
 x=struct('id',id,'success',r.success,'seconds',r.seconds,'oldSeconds',old.r.seconds,'validated',false,'sameSegments',false,'maxSegmentDifference',NaN,'oldLength',old.r.best.length,'length',NaN,'executionTime',NaN,'oldExecutionTime',old.r.execution.totalTime,'cusps',NaN);
 if r.success
  e=evalcw.profile(evalcw.pieces(r.best),scene.truck);c=evalcw.independent(scene,r.best);x.validated=e.valid&&c.passed;r.execution=e;r.independentCertificate=c;
  x.length=e.length;x.executionTime=e.totalTime;x.cusps=e.gearChanges;
  p=evalcw.pieces(r.best);b=evalcw.pieces(old.r.best);if isequal(size(p),size(b)),x.maxSegmentDifference=max(abs(p(:)-b(:)));x.sameSegments=x.maxSegmentDifference<1e-7;end
 end
 save(fullfile(out,[id '.mat']),'scene','r');rows=[rows;x];cb.json(fullfile(root,'results','reproduced','fast100_progress.json'),rows);
 fprintf('%s success%d valid%d same%d old%.3f new%.3f diff%.3g\n',id,x.success,x.validated,x.sameSegments,x.oldSeconds,x.seconds,x.maxSegmentDifference);
end
cb.json(fullfile(root,'results','reproduced','fast100.json'),rows);assert(all([rows.success])&&all([rows.validated]));
fprintf('REGRESSION COMPLETE: %d passed; same segments %d; mean old %.6f new %.6f\n',sum([rows.validated]),sum([rows.sameSegments]),mean([rows.oldSeconds]),mean([rows.seconds]));
end
