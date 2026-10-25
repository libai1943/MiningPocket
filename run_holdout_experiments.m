function rows=run_holdout_experiments()
if isfile(fullfile(fileparts(mfilename('fullpath')),'qa/fixed_regression_pending.flag')),error('MiningPocket:RegressionPending','Complete the fixed-task regression before independent tests.');end
root=miningpocket_setup();out=fullfile(root,'results/holdout');if ~isfolder(out),mkdir(out);end
rows=struct([]);methods={'BRS','HA','BIAGT','GHA','CC-A*'};
for i=1:12
 d=load(fullfile(root,'benchmark/holdout',sprintf('H%03d.mat',i)));scene=d.scene;
 for m=circshift(1:5,[0 mod(i-1,5)])
  method=methods{m};target=fullfile(out,[strrep(method,'*','star') '_' scene.id '.mat']);
  if isfile(target),z=load(target,'row');rows=[rows z.row];continue;end
  clear dp.link
  r=[];e=[];v=[];t=tic;c0=cputime;row=struct('id',scene.id,'method',method,'success',false,'J',NaN,'totalTime',NaN,'internalStops',NaN,'seconds',NaN,'cpuSeconds',NaN,'minimumResidual',NaN,'error','');
  try
   switch method
    case 'BRS',r=cm.solve(scene);
    case 'CC-A*',r=base.ccsearch(scene,base.options());
    case 'BIAGT',r=base.biagt(scene,base.options());
    otherwise,r=base.search(scene,method,base.options());
   end
   e=[];v=[];if r.success,[row.J,e]=evalcw.score(r.best,scene.truck);end
   row.seconds=toc(t);row.cpuSeconds=cputime-c0;
   if r.success
    v=evalcw.independent(scene,r.best);row.success=v.passed&&e.valid;row.totalTime=e.totalTime;row.internalStops=e.internalStops;row.minimumResidual=v.minimumResidual;
   end
  catch err,row.seconds=toc(t);row.cpuSeconds=cputime-c0;row.error=getReport(err,'extended','hyperlinks','off');end
  save(target,'r','e','v','row','-v7');rows=[rows row];evalcw.writeJson(fullfile(out,'holdout12.json'),rows);
  fprintf('%s %s valid=%d J=%.3f seconds=%.3f\n',method,scene.id,row.success,row.J,row.seconds);
 end
end
end
