function r=local(scene,o,zero)
% Evaluate the two complementary maneuver families at the first feasible scale.
timer=tic;r=struct('success',false,'best',[],'stage',0,'kind',0,'class',cc.classify(scene),'trials',{{}},'seconds',0,'scaleLevel',0,'J',Inf,'firstSolutionSeconds',NaN,'alternatives',{{}});
if nargin<3,od=dp.options();od.refine=o.refine;zero=dp.plan(scene,od);end
if zero.success
 sol=cm.solution({struct('path',zero.best,'gear',-1,'reverseOrder',true)},o.reverseWeight);
 accept(sol,0,0,0);
 if strcmp(o.branchPolicy,'first'),r.seconds=toc(timer);return;end
end
directFamily(-1,1,0);
if r.success&&strcmp(o.branchPolicy,'first'),r.seconds=toc(timer);return;end
alpha=abs(atan2(sin(scene.start(3)-scene.goal(3)),cos(scene.start(3)-scene.goal(3))));
if alpha>o.headingThreshold,order=[3 2];else,order=[2 3];end
if ~isempty(o.fixedOrder),order=o.fixedOrder;end
order=order(ismember(order,o.enabledBranches));
for level=1:o.scaleLevels
 current=o;if level>=2,current.fanRadiusFactors=o.expandedRadiusFactors;end
 if level==3,current.fanRampLengths=o.fineRampLength;current.fanAngles=o.fineAngles;current.connectorRampLengths=o.fineRampLength;current.connectorRadiusFactors=o.fineConnectorRadiusFactors;end
 for kind=order
  if cm.expired(o),break;end
  if kind==2&&level==1
   directFamily(1,2,level);
   if r.success&&strcmp(o.branchPolicy,'first'),break;end
  end
  trial=cm.one(scene,kind,current);
  r.trials{end+1}=struct('kind',kind,'scaleLevel',level,'success',trial.success,'fanCount',numel(trial.fans),'safeFans',sum(cellfun(@(x)x.certificate.passed,trial.fans)),'candidateCount',numel(trial.candidates),'checked',trial.checked,'seconds',trial.seconds,'feasibleCount',trial.feasibleCount);
  if trial.success,accept(trial.best,1,kind,level);end
  if r.success&&strcmp(o.branchPolicy,'first'),break;end
 end
 if r.success||cm.expired(o),break;end
end
r.seconds=toc(timer);
 function directFamily(gear,kind,level)
  if cm.expired(o),return;end
  tr=cm.direct(scene,gear,o);
  r.trials{end+1}=struct('kind',kind,'scaleLevel',level,'success',~isempty(tr.solutions),'fanCount',0,'safeFans',0,'candidateCount',tr.candidateCount,'checked',tr.checked,'seconds',tr.seconds,'feasibleCount',numel(tr.solutions),'construction','direct');
  for jj=1:numel(tr.solutions)
   if gear<0,accept(tr.solutions{jj},0,0,0);else,accept(tr.solutions{jj},1,2,level);end
  end
 end
 function accept(sol,stage,kind,level)
  [value,ex]=evalcw.score(sol,scene.truck);
  r.alternatives{end+1}=struct('kind',kind,'scaleLevel',level,'J',value,'length',sol.length,'extraStops',ex.internalStops);
  if ~r.success,r.firstSolutionSeconds=toc(timer);end
  if value<r.J-1e-9
   r.success=true;r.best=sol;r.stage=stage;r.kind=kind;r.scaleLevel=level;r.J=value;
  end
 end
end
