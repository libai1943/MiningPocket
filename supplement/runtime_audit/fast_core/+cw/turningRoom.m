function [available,probes]=turningRoom(scene,o,w)
% Two exact full-lock forward turns to goal heading, then one body length straight.
% This is a local space test, not a complete parking solution.
t=scene.truck;available=false;probes=struct('path',{},'q',{},'free',{});
extra=o.fineStep/2*(1+hypot(t.front,t.width/2)/t.radius);
for sign=[-1 1]
 angle=mod(sign*(scene.goal(3)-scene.start(3)),2*pi);if angle>2*pi-1e-10,angle=0;end
 p=struct('start',scene.start,'lengths',[t.radius*angle t.length],'curvatures',[sign/t.radius 0],'length',t.radius*angle+t.length,'word',0);
 q=cw.sample(p,o.fineStep);[hit,idx]=cw.collides(q,w,extra);if ~hit,idx=size(q,1);end
 probes(end+1)=struct('path',p,'q',q(1:min(idx,end),:),'free',~hit);available=available||~hit;
end
end
