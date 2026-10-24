function r=search(scene,g,o,incumbent)
% Lazy edge generation + Bellman recurrence on a DAG. Every ungenerated edge
% starts at an admissible Euclidean lower bound. Length only is additive.
if nargin<4,incumbent=[];end
timer=tic;E=numel(g.from);V=size(g.poses,1);weights=g.lower;state=zeros(1,E);paths=cell(1,E);index=ones(1,E);checks=0;generated=0;iterations=0;solved=false;best=[];edgePath=[];nodePath=[];lower=inf;
w=cw.world(scene,cw.defaults());incoming=cell(1,V);for e=1:E,incoming{g.to(e)}(end+1)=e;end
while generated+checks<o.maxEvaluations
 iterations=iterations+1;dist=inf(1,V);prev=zeros(1,V);dist(1)=0;
 for v=2:V
  ee=incoming{v};if isempty(ee),continue;end
  [value,j]=min(dist(g.from(ee))+weights(ee));dist(v)=value;prev(v)=ee(j);
 end
 lower=dist(end);if ~isfinite(lower),break;end
 route=[];v=V;while v~=1,e=prev(v);route=[e route];v=g.from(e);end
 unknown=route(state(route)==0);
 if ~isempty(unknown)
  for e=unknown
   a=g.poses(g.from(e),:);b=g.poses(g.to(e),:);
   if g.direct(e),pp=cc.connect(a,b,scene.truck.radius*o.radii,o.ramps,.98*pi);
   else,pp=dp.link(a,b,scene.truck.radius,4);end
   for k=1:numel(pp),pp{k}=dp.transform(pp{k},g.origin,g.axis);end
   paths{e}=pp;generated=generated+1;
   if isempty(pp),weights(e)=inf;state(e)=-1;else,weights(e)=pp{1}.length;state(e)=1;end
  end
  continue;
 end
 pending=route(state(route)==1);
 if ~isempty(pending)
  for e=pending
   cert=s1.check(scene,paths{e}{index(e)},o.checkStep,w);checks=checks+1;
   if cert.passed,state(e)=2;
   else,index(e)=index(e)+1;if index(e)>numel(paths{e}),state(e)=-1;weights(e)=inf;else,weights(e)=paths{e}{index(e)}.length;end;end
  end
  continue;
 end
 p=paths{route(1)}{index(route(1))};for e=route(2:end),p=cc.join(p,paths{e}{index(e)});end
 best=p;edgePath=route;nodePath=[1 g.to(route)];solved=true;break;
end
r=struct('success',solved,'best',best,'graph',g,'edgeState',state,'edgeWeights',weights,'edgePaths',{paths},'edgeChoice',index,'edgePath',edgePath,'nodePath',nodePath,'checks',checks,'generated',generated,'iterations',iterations,'lowerBound',lower,'certifiedGraphMinimum',solved,'seconds',toc(timer),'retainedCoarse',false);
if ~solved && ~isempty(incumbent) && incumbent.success
 r.success=true;r.best=incumbent.best;r.retainedCoarse=true;r.certifiedGraphMinimum=false;
end
end
