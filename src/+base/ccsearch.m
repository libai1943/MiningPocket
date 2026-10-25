function r=ccsearch(scene,o)
% Conventional pose-bin A* with the SAME zero-curvature steering connector.
if nargin<2,o=base.options();end
timer=tic;w=cw.world(scene,cw.defaults());R=scene.truck.radius;
tail=struct('path',cc.straight(scene.goal,scene.minimumStraight),'gear',-1,'reverseOrder',true);A=tail.path.goal;
r=struct('success',false,'best',[],'seconds',0,'expanded',0,'nodes',1,'candidateCount',0,'firstSolutionSeconds',NaN,'trace',zeros(0,3),'reason','budget');
tc=s1.check(scene,tail.path,.04,w);if ~tc.passed,return;end
q=scene.start;parents=0;legs={[]};gs=0;gears=0;cusps=0;
heap=base.Heap;heap.push(norm(q(1:2)-A(1:2)),1);
seen=containers.Map('KeyType','char','ValueType','double');seen(key(q,0,0))=0;
bestJ=Inf;expanded=0;nodes=1;hm=base.gridHeuristic(w,A,o.xyResolution);
while heap.n>0&&nodes<o.maxNodes&&toc(timer)<o.maxSeconds
 [id,~]=heap.pop();ky=key(q(id,:),gears(id),cusps(id));if seen(ky)<gs(id)-1e-8,continue;end
 expanded=expanded+1;
 if expanded==1||mod(expanded,o.shotInterval)==0
  prefix={};v=id;while parents(v)>0,prefix=[legs(v) prefix];v=parents(v);end
  for gear=[-1 1]
   if gear<0,pp=cc.connect(A,q(id,:),R*[1 1.3 1.7],[4 8],1.8*pi);
   else,pp=cc.connect(q(id,:),A,R*[1 1.3 1.7],[4 8],1.8*pi);end
   accepted=0;
   for j=1:numel(pp)
    if toc(timer)>o.maxSeconds,break;end
    c=s1.check(scene,pp{j},.04,w);if ~c.passed,continue;end
    closing=struct('path',pp{j},'gear',gear,'reverseOrder',gear<0);
    sol=cm.solution([prefix {closing tail}]);if sol.cusps>3,continue;end
    cert=cm.verify(scene,sol,.04);if ~cert.passed,continue;end
    [J,e]=evalcw.score(sol,scene.truck);r.candidateCount=r.candidateCount+1;accepted=accepted+1;
    if ~r.success,r.firstSolutionSeconds=toc(timer);r.firstExecution=e;r.firstBest=sol;r.firstJ=J;end
    if J<bestJ,bestJ=J;r.best=sol;r.best.certificate=cert;r.execution=e;r.success=true;r.trace(end+1,:)=[toc(timer) J r.candidateCount];end
    if accepted>=3,break;end
   end
  end
  if ~r.success
   sets=base.ccwordShots(q(id,:),A,scene,w,o,timer);
   for jj=1:numel(sets)
    sol=cm.solution([prefix sets{jj} {tail}]);if sol.cusps>3,continue;end
    cert=cm.verify(scene,sol,.04);if ~cert.passed,continue;end
    [J,e]=evalcw.score(sol,scene.truck);r.candidateCount=r.candidateCount+1;
    if ~r.success,r.firstSolutionSeconds=toc(timer);r.firstExecution=e;r.firstBest=sol;r.firstJ=J;end
    if J<bestJ,bestJ=J;r.best=sol;r.best.certificate=cert;r.execution=e;r.success=true;r.trace(end+1,:)=[toc(timer) J r.candidateCount];end
   end
  end
  if r.candidateCount>=o.ccCandidates,r.reason='candidate_budget';break;end
 end
 for L=[4 8 12]
  for gear=[1 -1]
   nc=cusps(id)+(gears(id)~=0&&gears(id)~=gear);if nc>3,continue;end
   for side=[-1 -.5 0 .5 1]
    if nodes>=o.maxNodes||toc(timer)>=o.maxSeconds,break;end
    if side==0
     start=q(id,:);start(3)=start(3)+(gear<0)*pi;p=cc.straight(start,L);p.bodyOffset=(gear<0)*pi;
     leg=struct('path',p,'gear',gear,'reverseOrder',false);
    else,leg=cm.prefix(q(id,:),gear,sign(side),L,R/abs(side),1);end
    qe=cc.endpoint(leg.path);qe(3)=qe(3)-leg.path.bodyOffset;
    value=gs(id)+L*(1+(o.reverseWeight-1)*(gear<0))+o.switchCost*(nc-cusps(id));
    ky=key(qe,gear,nc);if isKey(seen,ky)&&seen(ky)<=value+1e-8,continue;end
    c=s1.check(scene,leg.path,.04,w);if ~c.passed,continue;end
    nodes=nodes+1;q(nodes,:)=qe;parents(nodes,1)=id;legs{nodes,1}=leg;gs(nodes,1)=value;gears(nodes,1)=gear;cusps(nodes,1)=nc;seen(ky)=value;
    hd=max([norm(qe(1:2)-A(1:2)),R*abs(atan2(sin(qe(3)-A(3)),cos(qe(3)-A(3)))),base.gridValue(hm,qe,A)]);heap.push(value+hd,nodes);
   end
  end
 end
end
r.seconds=toc(timer);r.expanded=expanded;r.nodes=nodes;
 function k=key(p,g,nc)
  k=sprintf('%d,%d,%d,%d,%d',round(p(1)/o.xyResolution),round(p(2)/o.xyResolution),mod(round(p(3)/o.yawResolution),round(2*pi/o.yawResolution)),g,nc);
 end
end
