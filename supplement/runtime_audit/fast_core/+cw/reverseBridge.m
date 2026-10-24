function result=reverseBridge(scene,o,result)
% Meet a goal-out prefix with an anchor propagated backwards from the start.
% The resulting path is still one continuous all-reverse manoeuvre.
w=cw.world(scene,o);t=scene.truck;timer=tic;P={};
for i=1:numel(result.bridgeTrace.goalPaths),P{end+1}=fromSamples(result.bridgeTrace.goalPaths{i});end
B={};poses=[];fan={};
for k=[0 -1/t.radius 1/t.radius -1/(1.4*t.radius) 1/(1.4*t.radius) -1/(2.4*t.radius) 1/(2.4*t.radius)]
 q0=scene.start;q0(3)=q0(3)+pi;
 for d=2:2:50
  q=cw.segment(q0,d,k,o.fineStep);q(:,3)=q(:,3)-pi;
  if cw.collides(q,w,o.fineStep/2*(1+hypot(t.front,t.width/2)/t.radius)),break;end
  edge=struct('start',q(end,:),'lengths',d,'curvatures',-k,'length',d,'word',0);
  B{end+1}=edge;poses(end+1,:)=q(end,:);fan{end+1}=q;
 end
end
best=inf;if result.success,best=result.best.cost;end
pairs=[];
for i=1:numel(P)
 q=cw.sample(P{i},2);pose=q(end,:);
 for j=1:numel(B),lower=P{i}.length+B{j}.length+norm(pose(1:2)-poses(j,1:2));pairs(end+1,:)=[lower i j];end
end
if ~isempty(pairs),pairs=sortrows(pairs,1);end
examined=0;
for ii=1:min(size(pairs,1),10000)
 if o.reverseWeight*pairs(ii,1)>=best,break;end
 p=P{pairs(ii,2)};b=B{pairs(ii,3)};q=cw.sample(p,2);edges=cw.dubins(q(end,:),b.start,t.radius);examined=examined+1;
 for j=1:numel(edges)
  e=edges(j);total=p.length+e.length+b.length;if o.reverseWeight*total>=best,continue;end
  if e.length>norm(q(end,1:2)-b.start(1:2))+pi*2*t.radius,continue;end
  if ~cw.isFree(e,w,o),continue;end
  tail=p;tail.lengths=[p.lengths e.lengths b.lengths];tail.curvatures=[p.curvatures e.curvatures b.curvatures];tail.length=total;
  fq=cw.sample(tail,o.fineStep);extra=o.fineStep/2*(1+hypot(t.front,t.width/2)/t.radius);if cw.collides(fq,w,extra),continue;end
  assert(norm(fq(end,1:2)-scene.start(1:2))<1e-6);
  sol=struct('head',[],'tail',tail,'forward',zeros(0,3),'reverse',flipud(fq),'cusp',[],'cost',o.reverseWeight*total,'forwardLength',0,'reverseLength',total,'stem',scene.minimumStraight,'cuspCount',0);
  result.best=sol;result.feasible{end+1}=sol;result.success=true;result.status='SUCCESS';result.route=4;best=sol.cost;
 end
end
result.backwardTrace=struct('poses',poses,'fan',{fan},'pairsExamined',examined,'pairBudget',10000,'seconds',toc(timer));
end
function p=fromSamples(q)
d=vecnorm(diff(q(:,1:2)),2,2);a=atan2(sin(diff(q(:,3))),cos(diff(q(:,3))));len=d;ii=abs(a)>1e-10;len(ii)=d(ii).*a(ii)./(2*sin(a(ii)/2));k=a./max(len,eps);ls=[];ks=[];
for j=1:numel(len)
 if len(j)<1e-10,continue;end
 if ~isempty(ks)&&abs(k(j)-ks(end))<1e-8,ls(end)=ls(end)+len(j);else,ls(end+1)=len(j);ks(end+1)=k(j);end
end
p=struct('start',q(1,:),'lengths',ls,'curvatures',ks,'length',sum(ls),'word',0);
end
