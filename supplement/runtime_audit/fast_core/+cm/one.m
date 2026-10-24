function r=one(scene,kind,o,mode)
% Branch 2: grow the reverse tail from the goal. Branch 3: grow from start.
% Complete all anchor layers, then order full candidates by LF + w*LR.
if nargin<3,o=cm.options();end
if nargin<4,mode='ordered';end
timer=tic;w=cw.world(scene,cw.defaults());fans={};candidates={};R=scene.truck.radius;connectorRadii=R*o.fanRadiusFactors;if isfield(o,'connectorRadiusFactors'),connectorRadii=R*o.connectorRadiusFactors;end
if kind==2,stems=scene.minimumStraight+o.goalOffsets;root=scene.goal;else,stems=o.startOffsets;root=scene.start;end
last=cc.straight(scene.goal,scene.minimumStraight);
for ell=stems
 stem=cc.straight(root,ell);anchor=stem.goal;
 for radius=R*o.fanRadiusFactors
  for ramp=o.fanRampLengths
   for side=[-1 1]
    for angle=o.fanAngles
     turn=struct('start',anchor,'goal',[0 0 0],'segments',cc.turnSegments(side*angle,radius,ramp),'length',0);
     turn.length=sum(turn.segments(:,1));turn.goal=cc.endpoint(turn);fan=cc.join(stem,turn);
     cert=s1.check(scene,fan,o.step,w);
     fans{end+1}=struct('path',fan,'certificate',cert,'anchor',anchor,'stemLength',ell,'radius',radius,'ramp',ramp,'side',side,'angle',angle);
     fid=numel(fans);if ~cert.passed,continue;end
     if kind==2,conn=cc.connect(scene.start,fan.goal,connectorRadii,o.connectorRampLengths,o.connectorMaxTurn);
     else,conn=cc.connect(last.goal,fan.goal,connectorRadii,o.connectorRampLengths,o.connectorMaxTurn);end
     for k=1:numel(conn)
      if kind==2,head=conn{k};tail=fan;else,head=fan;tail=cc.join(last,conn{k});end
      legs={struct('path',head,'gear',1,'reverseOrder',false),struct('path',tail,'gear',-1,'reverseOrder',true)};
      sol=cm.solution(legs,o.reverseWeight);sol.fanIndex=fid;sol.connector=conn{k};sol.cusp=fan.goal;
      candidates{end+1}=sol;
     end
    end
   end
  end
 end
end
if ~isempty(candidates),cost=cellfun(@(x)x.cost,candidates);[~,ix]=sortrows([cost(:) (1:numel(cost))'],[1 2]);candidates=candidates(ix);end
generation=toc(timer);status=zeros(1,numel(candidates));certs=cell(1,numel(candidates));best=[];winner=0;checked=0;
for j=1:numel(candidates)
 sol=candidates{j};if kind==2,p=sol.legs{1}.path;else,p=sol.legs{2}.path;end
 cert=s1.check(scene,p,o.step,w);certs{j}=cert;checked=checked+1;
 if cert.passed
  status(j)=1;
  if isempty(best),best=sol;winner=j;end
  if strcmp(mode,'ordered'),break;end
 else,status(j)=-1;end
end
r=struct('success',~isempty(best),'kind',kind,'class',cc.classify(scene),'fans',{fans},'candidates',{candidates},'status',status,'certificates',{certs},'best',best,'winner',winner,'checked',checked,'generationSeconds',generation,'seconds',toc(timer),'options',o);
if r.success,r.best.certificate=cm.verify(scene,r.best,o.step);assert(r.best.certificate.passed);end
end
