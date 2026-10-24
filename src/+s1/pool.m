function [paths,meta]=pool(scene,o)
% Geometry first. Boundary/obstacle contents are not read here.
if nargin<2,o=s1.options();end
timer=tic;stem=cc.straight(scene.goal,scene.minimumStraight);a=stem.goal;b=scene.start;
R=scene.truck.radius*o.radiusFactors;Lc=o.rampLengths;paths={};
rawCount=0;seen=containers.Map('KeyType','char','ValueType','logical');
C=cc.connect(a,b,R,Lc,o.maxTurn);
for j=1:numel(C),append({C{j}},[]);end
directCount=numel(paths);poses=[];d=norm(b(1:2)-a(1:2));
if d>1e-8
 e=(b(1:2)-a(1:2))/d;n=[-e(2) e(1)];axisYaw=atan2(e(2),e(1));
 for t=o.stations
  for h=scene.truck.radius*o.offsetFactors
   for psi=o.headingOffsets
    mid=[a(1:2)+t*d*e+h*n axisYaw+psi];poses(end+1,:)=mid;
    left=cc.connect(a,mid,R,Lc,o.maxTurn);right=cc.connect(mid,b,R,Lc,o.maxTurn);
    for u=1:numel(left)
     for v=1:numel(right),append({left{u},right{v}},[t h psi]);end
    end
   end
  end
 end
end
if ~isempty(paths)
 cost=cellfun(@(p)p.length,paths);[~,ix]=sortrows([cost(:) (1:numel(paths))'],[1 2]);paths=paths(ix);
end
meta=struct('rawCount',rawCount,'count',numel(paths),'directCount',directCount,'poses',poses,'options',o,'generationSeconds',toc(timer));
 function append(links,parameters)
  p=stem;for k=1:numel(links),p=cc.join(p,links{k});end
  % Merge identical adjacent curvature laws (e.g. degenerate straight roots).
  seg=p.segments;clean=zeros(0,3);
  for k=1:size(seg,1)
   if seg(k,1)<1e-8,continue;end
   if ~isempty(clean) && abs(clean(end,3)-seg(k,3))<1e-12 && abs(clean(end,2)+clean(end,1)*clean(end,3)-seg(k,2))<1e-10
    clean(end,1)=clean(end,1)+seg(k,1);
   else,clean(end+1,:)=seg(k,:);end
  end
  p.segments=clean;p.length=sum(clean(:,1));rawCount=rawCount+1;
  signature=sprintf('%.7f,%.10f,%.12f;',clean');
  if isKey(seen,signature),return;end
  seen(signature)=true;p.links=links;p.poseParameters=parameters;p.generationId=rawCount;
  paths{end+1}=p;
 end
end
