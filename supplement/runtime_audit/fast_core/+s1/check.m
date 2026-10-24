function cert=check(scene,path,step,w)
% Same mathematical acceptance predicate as cc.check; reuse the map index.
if nargin<4,w=cw.world(scene,cw.defaults());end
[q,s,k]=cc.sample(path,step);rho=hypot(max(scene.truck.front,scene.truck.rear)+w.margin,scene.truck.width/2+w.margin);
K=max(abs(k));epsMotion=max(diff(s))/2*(1+rho*K);numericAllowance=1e-5;
physical=q;if isfield(path,'bodyOffset'),physical(:,3)=physical(:,3)-path.bodyOffset;end
subset=unique([1:25:size(q,1),size(q,1)]);
[hit,first]=cw.collides(physical(subset,:),w,0);confirmed=hit;
if hit,first=subset(first);else,[hit,first]=cw.collides(physical,w,epsMotion+numericAllowance);end
seg=path.segments;
if size(seg,1)>1,jump=max(abs(seg(1:end-1,2)+seg(1:end-1,3).*seg(1:end-1,1)-seg(2:end,2)));else,jump=0;end
endErr=norm(q(end,1:2)-path.goal(1:2));yawErr=abs(atan2(sin(q(end,3)-path.goal(3)),cos(q(end,3)-path.goal(3))));
cert=struct('passed',~hit && K<=1/scene.truck.radius+1e-8 && jump<1e-8 && endErr<1e-5 && yawErr<1e-7,'collision',hit,'confirmedCollision',confirmed,'first',first,'motionAllowance',epsMotion,'numericAllowance',numericAllowance,'maxCurvature',K,'maxSharpness',max(abs(seg(:,3))),'curvatureJump',jump,'endError',endErr,'yawError',yawErr,'length',path.length,'sampleCount',size(q,1));
end
