function r=local(scene,o,zero)
% Input angle selects the anchor direction. Every invocation first seeks zero cusps.
timer=tic;r=struct('success',false,'best',[],'stage',0,'kind',0,'class',cc.classify(scene),'trials',{{}},'seconds',0,'scaleLevel',0);
if nargin<3,zero=dp.plan(scene);end
if zero.success
 r.success=true;r.best=cm.solution({struct('path',zero.best,'gear',-1,'reverseOrder',true)},o.reverseWeight);r.stage=0;r.seconds=toc(timer);return
end
if r.class.class==3,order=[3 2];else,order=[2 3];end
for level=1:o.scaleLevels
 current=o;if level>=2,current.fanRadiusFactors=o.expandedRadiusFactors;end
 if level==3,current.fanRampLengths=o.fineRampLength;current.fanAngles=o.fineAngles;current.connectorRampLengths=o.fineRampLength;current.connectorRadiusFactors=o.fineConnectorRadiusFactors;end
 for kind=order
  trial=cm.one(scene,kind,current);r.trials{end+1}=struct('kind',kind,'scaleLevel',level,'success',trial.success,'fanCount',numel(trial.fans),'safeFans',sum(cellfun(@(x)x.certificate.passed,trial.fans)),'candidateCount',numel(trial.candidates),'checked',trial.checked,'seconds',trial.seconds);
  if trial.success,r.success=true;r.best=trial.best;r.stage=1;r.kind=kind;r.scaleLevel=level;break;end
 end
 if r.success,break;end
end
r.seconds=toc(timer);
end