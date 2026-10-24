function result=bridge(scene,o,result)
% Enumerate reusable goal-side and start-side anchors, then join analytically.
% No change of gear inside either tree; their shared pose is the only cusp.
timer=tic;w=cw.world(scene,o);t=scene.truck;g=scene.goal;L=scene.minimumStraight;
base=struct('start',g,'lengths',L,'curvatures',0,'length',L,'word',0);gq=cw.sample(base,o.step);a=gq(end,:);
trace=struct('goalAnchors',zeros(0,3),'startAnchors',zeros(0,3),'goalPaths',{{}},'startPaths',{{}},'joined',{{}},'tested',0,'method','two-root analytic anchor bridge');
% Directly reachable anchors: fixed steering, arclength and side samples.
P={base};S={};
radii=t.radius*[1 1.4 2.4];lengths=4:4:64;
for origin=1:2
 if origin==1,q0=a;pre=base;else,q0=scene.start;pre=[];end
 for k=[0 1./radii -1./radii]
  for d=lengths
   edge=struct('start',q0,'lengths',d,'curvatures',k,'length',d,'word',0);
   if ~cw.isFree(edge,w,o),break;end
   if isempty(pre),whole=edge;else,whole=join(pre,edge);end
   q=cw.sample(edge,2);pose=q(end,:);
   if origin==1,P{end+1}=whole;trace.goalAnchors(end+1,:)=pose;trace.goalPaths{end+1}=cw.sample(whole,.5);
   else,S{end+1}=whole;trace.startAnchors(end+1,:)=pose;trace.startPaths{end+1}=cw.sample(whole,.5);end
  end
 end
end
% Add map-guided analytic anchors on every 8 m station, across the corridor.
[path,~]=cw.guide(w.points,[],a(1:2),scene.start(1:2),t.width/2+o.margin,.75);
if ~isempty(path)
 ss=[0;cumsum(vecnorm(diff(path),2,2))];[ss,ix]=unique(ss);path=path(ix,:);
 for ds=unique([0:8:ss(end),ss(end)])
  p=interp1(ss,path,ds);p0=interp1(ss,path,max(0,ds-3));p1=interp1(ss,path,min(ss(end),ds+3));theta=atan2(p1(2)-p0(2),p1(1)-p0(1));
  for off=[0 -4 4 -8 8 -12 12]
   xy=p+off*[-sin(theta) cos(theta)];
   for da=deg2rad([0 -30 30 -60 60 90 -90 180])
    pose=[xy theta+da];if cw.collides(pose,w,.08),continue;end
    trace.tested=trace.tested+1;
    for origin=1:2
     if origin==1,q0=a;else,q0=scene.start;end
     edges=cw.dubins(q0,pose,t.radius);
     for j=1:numel(edges)
      e=edges(j);if e.length>2.2*norm(q0(1:2)-xy)+40,continue;end
      if ~cw.isFree(e,w,o),continue;end
      if origin==1,P{end+1}=join(base,e);trace.goalAnchors(end+1,:)=pose;trace.goalPaths{end+1}=cw.sample(P{end},.5);
      else,S{end+1}=e;trace.startAnchors(end+1,:)=pose;trace.startPaths{end+1}=cw.sample(e,.5);end
      break;
     end
    end
   end
  end
 end
end
fprintf('%s anchors goal=%d start=%d %.1fs\n',scene.id,numel(P),numel(S),toc(timer));
% First try an all-reverse path, with one intermediate analytic anchor.
best=inf;if result.success,best=result.best.cost;end
for ip=1:numel(P)
 pp=cw.sample(P{ip},2);q=pp(end,:);edges=cw.dubins(q,scene.start,t.radius);
 for j=1:numel(edges)
  e=edges(j);if o.reverseWeight*(P{ip}.length+e.length)>=best,continue;end
  if ~cw.isFree(e,w,o),continue;end
  tail=join(P{ip},e);[result,best,ok]=accept(result,[],tail,scene,o,w,best);
  if ok,trace.joined{end+1}=result.best;end
 end
end
% Enumerated forward arcs end at exact cusp poses. Connect goal anchors to them.
% Tail lower bounds order pairs; the result is independent of grid snapping.
Q=zeros(numel(S),3);for j=1:numel(S),q=cw.sample(S{j},2);Q(j,:)=q(end,:);end
pairs=[];for ip=1:numel(P)
 pp=cw.sample(P{ip},2);q=pp(end,:);
 for js=1:numel(S)
  lower=S{js}.length+o.reverseWeight*(P{ip}.length+norm(q(1:2)-Q(js,1:2)));
  if lower<best,pairs(end+1,:)=[lower ip js];end
 end
end
if ~isempty(pairs),pairs=sortrows(pairs,1);end
z=0;for z=1:size(pairs,1)
 if pairs(z,1)>=best,break;end
 ip=pairs(z,2);js=pairs(z,3);pp=cw.sample(P{ip},2);q=pp(end,:);
 edges=cw.dubins(q,Q(js,:),t.radius);
 for j=1:numel(edges)
  e=edges(j);trace.tested=trace.tested+1;
  if S{js}.length+o.reverseWeight*(P{ip}.length+e.length)>=best,continue;end
  if e.length>norm(q(1:2)-Q(js,1:2))+pi*2*t.radius,continue;end
  if ~cw.isFree(e,w,o),continue;end
  tail=join(P{ip},e);[result,best,ok]=accept(result,S{js},tail,scene,o,w,best);
  if ok,trace.joined{end+1}=result.best;end
 end
 if isfinite(best)&&z>=1000,break;end
 if z>=8000,break;end
end
trace.pairBudget=8000;trace.pairsExamined=z;if isempty(z),trace.pairsExamined=0;end;result.bridgeTrace=trace;result.bridgeSeconds=toc(timer);result.bridgePairCount=size(pairs,1);
if ~isempty(result.best),result.success=true;result.status='SUCCESS';result.route=4;end
end
function c=join(a,b)
c=a;c.lengths=[a.lengths b.lengths];c.curvatures=[a.curvatures b.curvatures];c.length=a.length+b.length;
end
function [r,best,ok]=accept(r,head,tail,s,o,w,best)
ok=false;extra=o.fineStep/2*(1+hypot(s.truck.front,s.truck.width/2)/s.truck.radius);b=cw.sample(tail,o.fineStep);
if cw.collides(b,w,extra),return;end
if isempty(head),f=zeros(0,3);fl=0;cusp=[];else,f=cw.sample(head,o.fineStep);fl=head.length;cusp=f(end,:);if cw.collides(f,w,extra),return;end;end
cost=fl+o.reverseWeight*tail.length;if cost>=best,return;end
sol=struct('head',head,'tail',tail,'forward',f,'reverse',flipud(b),'cusp',cusp,'cost',cost,'forwardLength',fl,'reverseLength',tail.length,'stem',tail.lengths(1),'cuspCount',~isempty(head));
r.best=sol;r.feasible{end+1}=sol;best=cost;ok=true;
end
