function export_paper_data(paper)
root=miningpocket_setup();out=fullfile(root,'results/validated');
if nargin<1,paper=fullfile(root,'results/paper_data');end
for folder={'analysis','inputs'},target=fullfile(paper,folder{1});if ~isfolder(target),mkdir(target);end;end
audit=struct([]);first=struct([]);
for i=1:100
 id=sprintf('B%03d',i);z=load(fullfile(out,['BRS_' id '.mat']),'r','row');r=z.r;
 a=struct('id',id,'success',z.row.success,'rootSuccess',r.root.success,'branch',z.row.branch,'rootSeconds',r.root.seconds,'selectedJ',z.row.J,'coarseJ',r.zero.coarseJ,'fineJ',r.zero.fineJ,'fineAttempted',~isempty(r.zero.fine),'selectedFine',r.zero.selectedFine,'alternatives',{r.root.alternatives},'firstJ',NaN,'firstSeconds',r.firstSolutionSeconds,'totalSeconds',z.row.totalPlanningSeconds,'recursive',z.row.recursive);
 if r.root.success && ~isempty(r.root.alternatives),a.firstJ=r.root.alternatives{1}.J;
 elseif r.success
  for j=1:numel(r.nodes)
   n=r.nodes{j};if ~n.success,continue;end
   prefix={n.leg};parent=n.parent;
   while parent>0,prefix=[{r.nodes{parent}.leg} prefix];parent=r.nodes{parent}.parent;end
   sol=cm.solution([prefix n.local.best.legs]);a.firstJ=evalcw.score(sol,loadTruck(root,id));break
  end
 end
 a.rootCandidates=0;for k=1:numel(r.root.trials),a.rootCandidates=a.rootCandidates+r.root.trials{k}.candidateCount;end
 a.candidates=a.rootCandidates;
 for k=1:numel(r.nodes),if r.nodes{k}.tested,tt=r.nodes{k}.local.trials;for j=1:numel(tt),a.candidates=a.candidates+tt{j}.candidateCount;end;end;end
 audit=[audit a];
 for method={'BRS','HA','BIAGT','GHA','CC-A*'}
  m=method{1};v=load(fullfile(out,[strrep(m,'*','star') '_' id '.mat']),'r','row');rr=v.r;
  fj=NaN;if strcmp(m,'BRS'),fj=a.firstJ;elseif isfield(rr,'firstJ'),fj=rr.firstJ;elseif v.row.success,fj=v.row.J;end
  first=[first struct('id',id,'method',m,'success',v.row.success,'firstJ',fj,'finalJ',v.row.J,'firstSeconds',rr.firstSolutionSeconds,'finalSeconds',v.row.totalPlanningSeconds)];
 end
end
evalcw.writeJson(fullfile(out,'selection_audit.json'),audit);evalcw.writeJson(fullfile(out,'first_final.json'),first);
ids={'B003','B058','B018','B055','B027','B030','B081','B100'};
for j=1:numel(ids)
 id=ids{j};v=load(fullfile(out,['BRS_' id '.mat']));r=v.r;d=load(fullfile(root,'benchmark/cases',[id '.mat']));sc=d.scene;
 if ~r.success,continue;end
 z=struct('id',id,'scene',sc,'root',rmfield(r.root,'best'),'legs',{exportLegs(r.best.legs)},'execution',v.e,'nodes',[],'dp',[],'fan',[],'prefixLegCount',0);
 if ~isempty(r.zero.coarse)
  a=r.zero.coarse;g=a.graph;po=g.poses;rot=[cos(g.axis) -sin(g.axis);sin(g.axis) cos(g.axis)];po(:,1:2)=po(:,1:2)*rot'+g.origin;po(:,3)=po(:,3)+g.axis;
  z.dp.axis=g.axis;z.dp.edgeCurves=struct([]);
  for ee=1:numel(a.edgePaths)
   pp=a.edgePaths{ee};if isempty(pp),continue;end
   kk=max(1,min(a.edgeChoice(ee),numel(pp)));qq=cc.sample(pp{kk},.4);z.dp.edgeCurves=[z.dp.edgeCurves struct('q',qq,'state',a.edgeState(ee))];
  end
  z.dp.finePoses=[];if ~isempty(r.zero.fine),gg=r.zero.fine.graph;fp=gg.poses;fp(:,1:2)=fp(:,1:2)*rot'+g.origin;fp(:,3)=fp(:,3)+g.axis;z.dp.finePoses=fp;end
  z.dp.poses=po;z.dp.from=g.from;z.dp.to=g.to;z.dp.state=a.edgeState;z.dp.nodePath=a.nodePath;z.dp.edgePath=a.edgePath;z.dp.coarse=[];z.dp.fine=[];
  if a.success,[q,s,k]=cc.sample(a.best,.15);z.dp.coarse=struct('q',q,'s',s,'k',k);end
  if ~isempty(r.zero.fine)&&r.zero.fine.success,[q,s,k]=cc.sample(r.zero.fine.best,.15);z.dp.fine=struct('q',q,'s',s,'k',k);end
 end
 if strcmp(id,'B058') && r.zero.success
  z.legs=exportLegs({struct('path',r.zero.best,'gear',-1,'reverseOrder',true)});
 end
 if any(strcmp(id,{'B018','B055'}))
  o=cm.options();kind=2+strcmp(id,'B055');t=cm.one(sc,kind,o);assert(t.success);
  % This panel illustrates the named branch, including its own selected output.
  z.legs=exportLegs(t.best.legs);z.fan=struct('kind',kind,'fans',[],'winner',t.winner,'checked',t.checked,'candidateCount',numel(t.candidates),'cusp',t.best.cusp,'connector',t.best.connector);
  for k=1:numel(t.fans),f=t.fans{k};q=cc.sample(f.path,.4);z.fan.fans=[z.fan.fans struct('q',q,'safe',f.certificate.passed,'anchor',f.anchor,'angle',f.angle,'radius',f.radius,'ramp',f.ramp)];end
 end
 if ~r.root.success
  nn=struct([]);
  for k=1:numel(r.nodes)
   n=r.nodes{k};[q,~,~]=cm.execution(n.leg,.15);
   nn=[nn struct('index',k,'parent',n.parent,'depth',n.depth,'q',q,'safe',n.certificate.passed,'success',n.success,'reason',n.reason,'totalCusps',n.totalCusps)];
  end
  z.nodes=nn;
  bestDifference=Inf;
  for kk=1:numel(r.nodes)
   node=r.nodes{kk};if ~node.success,continue;end
   prefix={node.leg};parent=node.parent;
   while parent>0,prefix=[{r.nodes{parent}.leg} prefix];parent=r.nodes{parent}.parent;end
   candidate=cm.solution([prefix node.local.best.legs]);value=evalcw.score(candidate,sc.truck);difference=abs(value-v.row.J);
   if difference<bestDifference,bestDifference=difference;z.prefixLegCount=numel(prefix);end
  end
  assert(bestDifference<1e-6);
 end
 evalcw.writeJson(fullfile(paper,'analysis',[id '_trace.json']),z);
end
d=load(fullfile(root,'benchmark/cases/B003.mat'));z=struct('scene',d.scene,'methods',[]);
for method={'BRS','HA','BIAGT','GHA','CC-A*'}
 m=method{1};d=load(fullfile(out,[strrep(m,'*','star') '_B003.mat']));q=[];s=[];k=[];off=0;
 for j=1:numel(d.r.best.legs)
  [qq,ss,kk]=cm.execution(d.r.best.legs{j},.15);q=[q;qq];s=[s;ss+off];k=[k;kk];off=s(end);
 end
 z.methods=[z.methods struct('method',m,'q',q,'s',s,'k',k,'events',d.e.events,'length',d.e.length)];
end
evalcw.writeJson(fullfile(paper,'inputs/B003_comparison.json'),z);
fprintf('Exported current experiment geometry and all selection records.\n');
end
function t=loadTruck(root,id)
d=load(fullfile(root,'benchmark/cases',[id '.mat']));t=d.scene.truck;
end
function out=exportLegs(legs)
out=struct([]);
for j=1:numel(legs),l=legs{j};[q,s,k]=cm.execution(l,.1);out=[out struct('q',q,'s',s,'k',k,'gear',l.gear,'length',l.path.length)];end
end

