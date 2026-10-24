function r=biagt(scene,o)
% Wang, Hansen and Ahn, TCST 2024, Algorithms 1-2, static part.
% Exact RS bridges implement the end-game; all maps/truck data come from CW100.
if nargin<2,o=base.options();end
timer=tic;w=cw.world(scene,cw.defaults());R=scene.truck.radius;A=scene.goal+[scene.minimumStraight*cos(scene.goal(3)),scene.minimumStraight*sin(scene.goal(3)),0];tail=[scene.minimumStraight 0 0 -1];
r=struct('success',false,'method','BIAGT','best',[],'seconds',0,'expanded',0,'nodes',2,'candidateCount',0,'reason','budget','firstSolutionSeconds',NaN);
T={init(scene.start,A,o,R),init(A,scene.start,o,R)};active=2;closeTrees=false;expanded=0;answer=[];
[tt,~]=base.primitive(A,scene.minimumStraight,0,-1,o.step);if ~base.free(tt,w,0,scene.minimumStraight/(size(tt,1)-1)),r.reason='terminal_blocked';r.seconds=toc(timer);return;end
while expanded<o.maxNodes&&toc(timer)<o.maxSeconds
 if T{1}.heap.n==0&&T{2}.heap.n==0,r.reason='exhausted';break;end
 if T{1}.heap.n==0,active=2;elseif T{2}.heap.n==0,active=1;elseif ~closeTrees,active=3-active;else
  [a,~]=T{1}.heap.peek();[b,~]=T{2}.heap.peek();active=1+(T{2}.h(b)<T{1}.h(a));
 end
 other=3-active;ta=T{active};tb=T{other};[id,~]=ta.heap.pop();q0=ta.q(id,:);expanded=expanded+1;
 if ta.untried(id,1)&&ta.untried(id,2)
  ds=metric(tb.q,q0,R);[sorted,ix]=sort(ds);targets=ix(1:min(3,numel(ix)));
  if min(ds)<=o.biMu,closeTrees=true;end
  if expanded<=2||mod(expanded,o.shotInterval)==0||sorted(1)<o.biDelta*2
   targets=targets(ds(targets)<=o.biMu);if ds(1)<=o.biMu,targets=unique([1;targets(:)],'stable');end
   for jj=targets'
    if active==1,aa=q0;bb=tb.q(jj,:);else,aa=tb.q(jj,:);bb=q0;end
    [ok,connector]=base.shot(aa,bb,w,o,'rs');
    if ok
     if active==1,ps=base.trace(ta.q,ta.parent,ta.move,id);pg=base.trace(tb.q,tb.parent,tb.move,jj);else,ps=base.trace(tb.q,tb.parent,tb.move,jj);pg=base.trace(ta.q,ta.parent,ta.move,id);end
     answer=[ps;connector;base.invert(pg);tail];break;
    end
   end
   if ~isempty(answer),r.reason='solved';break;end
  end
 end
 if all(ta.untried(id,:))&&ta.parent(id)>0,ta.priority(id,:)=ta.priority(ta.parent(id),:);end
 priorities=ta.priority(id,:);priorities(~ta.untried(id,:))=Inf;[~,mode]=min(priorities);gear=3-2*mode;ta.untried(id,mode)=false;newF=[];
 for kk=o.steers/R
  [qq,p]=base.primitive(q0,o.primitive,kk,gear,o.step);qe=qq(end,:);
  if min(metric(ta.q,qe,R))<o.biDelta,continue;end
  if ~base.free(qq,w,kk,o.primitive/(size(qq,1)-1)),continue;end
  phys=p;lg=ta.move(id,4);if active==2,phys(:,4)=-phys(:,4);lg=-lg;end
  gg=ta.g(id)+base.cost(phys,o,lg,ta.move(id,2));
  dd=metric(tb.q,qe,R);near=find(dd<o.biGamma);
  if isempty(near),hh=base.heuristic(qe,ta.target,R);else
   lb=vecnorm(tb.q(near,1:2)-qe(1:2),2,2)+tb.g(near);[lb,ord]=sort(lb);near=near(ord);hh=Inf;
   for jj=1:o.biNeighbours:numel(near),if lb(jj)>=hh,break;end;ix=near(jj:min(jj+o.biNeighbours-1,numel(near)));hh=min(hh,min(base.heuristic(repmat(qe,numel(ix),1),tb.q(ix,:),R)+tb.g(ix)));end
  end
  if min(dd)<=o.biMu,closeTrees=true;end
  ff=gg+o.biInflation*hh;n=size(ta.q,1)+1;ta.q(n,:)=qe;ta.parent(n,1)=id;ta.move(n,:)=p;ta.g(n,1)=gg;ta.h(n,1)=hh;ta.priority(n,:)=ta.priority(id,:);ta.untried(n,:)=[true true];ta.heap.push(ff,n);newF(end+1)=ff; %#ok<AGROW>
 end
 if ~isempty(newF),ta.priority(id,mode)=min(newF);end
 if any(ta.untried(id,:)),ta.heap.push(ta.g(id)+o.biInflation*ta.h(id),id);end
 T{active}=ta;
end
if ~isempty(answer)
 sol=base.build(scene.start,answer);cert=base.validate(scene,sol,.04);r.success=cert.passed;r.best=sol;r.best.certificate=cert;r.execution=evalcw.profile(answer,scene.truck,evalcw.options());r.candidateCount=1;r.firstSolutionSeconds=toc(timer);
end
r.seconds=toc(timer);r.expanded=expanded;r.nodes=size(T{1}.q,1)+size(T{2}.q,1);
end
function t=init(q,target,o,R)
t=struct('q',q,'target',target,'parent',0,'move',zeros(1,4),'g',0,'h',base.heuristic(q,target,R),'priority',[0 0],'untried',[true true],'heap',base.Heap);t.heap.push(o.biInflation*t.h,1);
end
function d=metric(q,p,R)
d=sqrt(sum((q(:,1:2)-p(:,1:2)).^2,2)+(R*atan2(sin(q(:,3)-p(:,3)),cos(q(:,3)-p(:,3)))).^2);
end

