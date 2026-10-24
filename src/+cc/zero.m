function r=zero(scene,withAnchors)
if nargin<2,withAnchors=true;end
timer=tic;r=struct('success',false,'status','BUDGET_NO_CONNECTION','class',cc.classify(scene),'paths',{{}},'certificates',{{}},'directCount',0,'anchors',[],'best',[],'seconds',0);
stem=cc.straight(scene.goal,scene.minimumStraight);a=stem.goal;b=scene.start;
paths=cc.connect(a,b,scene.truck.radius*[1 1.3 1.7],[4 8],pi*.98);
for j=1:numel(paths),accept(cc.join(stem,paths{j}));end
r.directCount=numel(r.paths);
% Obstacle-directed intermediate poses, rather than arbitrary lateral shifts.
if withAnchors
 info=cc.obstacleAnchors(scene,a,b);r.anchors=info;
 for j=1:size(info.anchors,1)
  mid=info.anchors(j,:);
  for dyaw=deg2rad([0 -15 15])
   mid(3)=info.anchors(j,3)+dyaw;
   left=cc.connect(a,mid,scene.truck.radius*[1 1.3],[4],pi*.95);
   right=cc.connect(mid,b,scene.truck.radius*[1 1.3],[4],pi*.95);
   left=left(1:min(3,numel(left)));right=right(1:min(3,numel(right)));
   for u=1:numel(left)
    for v=1:numel(right),accept(cc.join(stem,cc.join(left{u},right{v})));end
   end
  end
 end
end
if ~isempty(r.best),r.success=true;r.status='SUCCESS';end
r.seconds=toc(timer);
 function accept(p)
  p.goal=scene.start;cert=cc.check(scene,p,.15);
  r.paths{end+1}=p;r.certificates{end+1}=cert;
  if cert.passed && (isempty(r.best)||p.length<r.best.length)
   fine=cc.check(scene,p,.04);if fine.passed,r.best=p;r.best.certificate=fine;end
  end
 end
end
