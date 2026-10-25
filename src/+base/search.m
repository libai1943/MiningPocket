function r=search(scene,method,o)
if nargin<3,o=base.options();end
timer=tic;w=cw.world(scene,cw.defaults());R=scene.truck.radius;A=scene.goal+[scene.minimumStraight*cos(scene.goal(3)),scene.minimumStraight*sin(scene.goal(3)),0];tail=[scene.minimumStraight 0 0 -1];
reverse=strcmp(method,'GHA');if reverse,start=A;goal=scene.start;else,start=scene.start;goal=A;end
r=struct('success',false,'method',method,'best',[],'seconds',0,'expanded',0,'nodes',1,'candidateCount',0,'reason','budget','firstSolutionSeconds',NaN);
[tt,~]=base.primitive(A,scene.minimumStraight,0,-1,o.step);if ~base.free(tt,w,0,scene.minimumStraight/(size(tt,1)-1)),r.reason='terminal_blocked';r.seconds=toc(timer);return;end
q=zeros(o.maxNodes,3);parent=zeros(o.maxNodes,1);move=zeros(o.maxNodes,4);g=Inf(o.maxNodes,1);h=zeros(o.maxNodes,1);q(1,:)=start;g(1)=0;
if ~reverse,hm=base.gridHeuristic(w,goal,o.xyResolution);h(1)=max(base.heuristic(start,goal,R),base.gridValue(hm,start,goal));end
heap=base.Heap;heap.push(g(1)+h(1),1);bestAt=containers.Map('KeyType','char','ValueType','double');bestAt(key(start,0,o))=0;n=1;expanded=0;bestValue=Inf;
while heap.n>0&&n<o.maxNodes&&toc(timer)<o.maxSeconds
 [id,f]=heap.pop();if f>g(id)+h(id)+1e-8,continue;end
 stateKey=key(q(id,:),move(id,4),o);if bestAt(stateKey)<g(id)-1e-8,continue;end
 expanded=expanded+1;
 if expanded==1 || mod(expanded,o.shotInterval)==0
  if reverse,connSet=base.dubinsShots(q(id,:),goal,w,o);else,[ok,conn]=base.shot(q(id,:),goal,w,o,'rs',move(id,4),move(id,2));connSet={};if ok,connSet={conn};end;end
  stop=false;
  for candidate=1:numel(connSet)
   p=base.trace(q,parent,move,id);p=[p;connSet{candidate}];if reverse,p=base.invert(p);end;p=[p;tail];sol=base.build(scene.start,p);
   cert=base.validate(scene,sol,.04);
   if cert.passed
    e=evalcw.profile(p,scene.truck,evalcw.options());r.candidateCount=r.candidateCount+1;
    if ~r.success,r.firstSolutionSeconds=toc(timer);r.firstExecution=e;r.firstBest=sol;eoFirst=evalcw.options();r.firstJ=e.totalTime+eoFirst.stopPenalty*e.internalStops;end
    eo=evalcw.options();value=e.totalTime+eo.stopPenalty*e.internalStops;if value<bestValue,r.success=true;bestValue=value;r.best=sol;r.best.certificate=cert;r.execution=e;end
    if ~reverse||r.candidateCount>=o.goalCandidates,r.reason='solved';stop=true;break;end
   end
  end
  if stop,break;end
 end
 childQ=[];childP=[];childC=[];
 for gear=[1 -1]
  for kk=o.steers/R
   [qq,p]=base.primitive(q(id,:),o.primitive,kk,gear,o.step);
   physical=p;if reverse,physical(:,4)=-physical(:,4);lastGear=-move(id,4);else,lastGear=move(id,4);end
   cost=g(id)+base.cost(physical,o,lastGear,move(id,2));qqEnd=qq(end,:);ky=key(qqEnd,gear,o);
   if isKey(bestAt,ky)&&bestAt(ky)<=cost+1e-8,continue;end
   if ~base.free(qq,w,kk,o.primitive/(size(qq,1)-1)),continue;end
   childQ(end+1,:)=qqEnd;childP(end+1,:)=p;childC(end+1,1)=cost; %#ok<AGROW>
  end
 end
 if isempty(childQ),continue;end
 if reverse,hh=zeros(size(childC));else,hh=max(base.heuristic(childQ,goal,R),base.gridValue(hm,childQ,goal));end
 for j=1:size(childQ,1)
  if n>=o.maxNodes,break;end
  ky=key(childQ(j,:),childP(j,4),o);if isKey(bestAt,ky)&&bestAt(ky)<=childC(j)+1e-8,continue;end
  n=n+1;q(n,:)=childQ(j,:);parent(n)=id;move(n,:)=childP(j,:);g(n)=childC(j);h(n)=hh(j);bestAt(ky)=g(n);heap.push(g(n)+h(n),n);
 end
end
r.seconds=toc(timer);r.expanded=expanded;r.nodes=n;if r.success&&~strcmp(r.reason,'solved'),r.reason='budget_with_incumbent';elseif heap.n==0,r.reason='exhausted';end
end
function k=key(q,gear,o)
a=round(q(1)/o.xyResolution);b=round(q(2)/o.xyResolution);c=mod(round(q(3)/o.yawResolution),round(2*pi/o.yawResolution));k=sprintf('%d,%d,%d,%d',a,b,c,gear);
end


