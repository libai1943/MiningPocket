function rows=run_paper_experiments(caseNumbers,methods,outputDirectory)
% Sequential, deterministic order rotation; no stored paths enter planning.
if nargin<1,caseNumbers=1:100;end
if nargin<2,methods={'BRS','HA','BIAGT','GHA','CC-A*'};end
if nargin<3,outputDirectory='results/validated';end
root=miningpocket_setup();out=fullfile(root,outputDirectory);if ~isfolder(out),mkdir(out);end
rows=struct([]);summary=fullfile(out,'comparison100.json');if isfile(summary),rows=jsondecode(fileread(summary));rows=rows(:)';end
% Warm code and quadrature initialization, without retaining any scene cache.
cc.turn(.1,15,4);base.primitive([0 0 0],3,0,1,.1);
for i=caseNumbers
 d=load(fullfile(root,'benchmark/cases',sprintf('B%03d.mat',i)));scene=d.scene;
 order=circshift(1:numel(methods),[0 mod(i-1,numel(methods))]);
 for m=order
  method=methods{m};if ~isempty(rows)&&any(strcmp({rows.id},scene.id)&strcmp({rows.method},method)),continue;end
  clear dp.link
  row=struct('method',method,'id',scene.id,'mapGroup',scene.mapGroup,'originalId',scene.originalId,'success',false,'seconds',NaN,'length',NaN,'totalTime',NaN,'movingTime',NaN,'dwellTime',NaN,'cusps',NaN,'steeringStops',NaN,'internalStops',NaN,'candidateCount',0,'reason','','class',0,'firstSolutionSeconds',NaN,'parameterizationSeconds',0,'totalPlanningSeconds',NaN,'cpuSeconds',NaN,'J',NaN,'startedAt','','finishedAt','','branch',-1,'recursive',false,'checks',0,'minimumResidual',NaN,'poseError',[NaN NaN],'error','');
  row.startedAt=datestr(now,'yyyy-mm-ddTHH:MM:SS.FFF');t=tic;c0=cputime;r=[];e=[];validation=[];
  try
   switch method
    case 'BRS',r=cm.solve(scene);
    case 'CC-A*',r=base.ccsearch(scene,base.options());
    case 'BIAGT',r=base.biagt(scene,base.options());
    otherwise,r=base.search(scene,method,base.options());
   end
   row.seconds=toc(t);row.firstSolutionSeconds=r.firstSolutionSeconds;
   if isfield(r,'candidateCount'),row.candidateCount=r.candidateCount;end
   if r.success
    tp=tic;[row.J,e]=evalcw.score(r.best,scene.truck);row.parameterizationSeconds=toc(tp);
   end
   row.totalPlanningSeconds=toc(t);row.cpuSeconds=cputime-c0;row.finishedAt=datestr(now,'yyyy-mm-ddTHH:MM:SS.FFF');
   if r.success
    validation=evalcw.independent(scene,r.best);row.success=validation.passed&&e.valid;
    row.length=e.length;row.totalTime=e.totalTime;row.movingTime=e.movingTime;row.dwellTime=e.dwellTime;row.cusps=e.gearChanges;row.steeringStops=e.sameGearSteeringStops;row.internalStops=e.internalStops;
    row.checks=validation.samples;row.minimumResidual=validation.minimumResidual;row.poseError=validation.poseError;
   end
   cl=cc.classify(scene);row.class=cl.class;
   if strcmp(method,'BRS')
    row.recursive=~r.root.success;row.branch=r.root.kind;if r.root.stage==0&&r.root.success,row.branch=1;end
    for k=1:numel(r.root.trials),row.candidateCount=row.candidateCount+r.root.trials{k}.candidateCount;end
    row.reason=r.status;
   else,row.reason=r.reason;end
  catch err
   row.totalPlanningSeconds=toc(t);row.cpuSeconds=cputime-c0;row.finishedAt=datestr(now,'yyyy-mm-ddTHH:MM:SS.FFF');row.error=getReport(err,'extended','hyperlinks','off');
  end
  filename=[strrep(method,'*','star') '_' scene.id '.mat'];save(fullfile(out,filename),'r','e','validation','row','-v7');
  if isempty(rows),rows=row;else,rows(end+1)=row;end
  evalcw.writeJson(summary,rows);
  fprintf('%s %s valid=%d J=%.4f wall=%.3f CPU=%.3f %s\n',method,scene.id,row.success,row.J,row.totalPlanningSeconds,row.cpuSeconds,row.error);
 end
end
end
