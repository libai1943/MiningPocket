function rows=validate_recorded(caseNumbers)
if nargin<1,caseNumbers=1:100;end
root=miningpocket_setup(true);methods={'BRS','HA','BIAGT','GHA','CC-Astar'};rows=struct([]);
for m=1:numel(methods)
 for i=caseNumbers
  id=sprintf('B%03d',i);d=load(fullfile(root,'benchmark/cases',[id '.mat']));v=load(fullfile(root,'results/recorded',[methods{m} '_' id '.mat']));r=v.r;
  row=struct('id',id,'method',methods{m},'plannerSuccess',v.row.success,'passed',false,'J',NaN,'positionError',NaN,'headingError',NaN);
  if r.success
   a=evalcw.independent(d.scene,r.best);[row.J,e]=evalcw.score(r.best,d.scene.truck);row.passed=a.passed&&e.valid;row.positionError=a.poseError(1);row.headingError=a.poseError(2);
   assert(~v.row.success||row.passed,'A stored successful trajectory failed validation.');
  end
  rows=[rows row];fprintf('audit %s %s: planned=%d checked=%d\n',methods{m},id,r.success,row.passed);
 end
end
evalcw.writeJson(fullfile(root,'results/reproduced/independent_audit.json'),rows);
end
