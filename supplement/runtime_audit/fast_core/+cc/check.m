function cert=check(scene,path,step)
% Independent point-cloud / map-domain swept-body check with motion allowance.
if nargin<3,step=.08;end
o=cw.defaults();w=cw.world(scene,o);[q,s,k]=cc.sample(path,step);
rho=hypot(scene.truck.front,scene.truck.width/2);ds=max(diff(s));K=max(abs(k));
epsMotion=ds/2*(1+rho*K);numericAllowance=1e-5;
physical=q;if isfield(path,'bodyOffset'),physical(:,3)=physical(:,3)-path.bodyOffset;end
[hit,first]=cw.collides(physical,w,epsMotion+numericAllowance);
seg=path.segments;
if size(seg,1)>1,jump=max(abs(seg(1:end-1,2)+seg(1:end-1,3).*seg(1:end-1,1)-seg(2:end,2)));else,jump=0;end
endErr=norm(q(end,1:2)-path.goal(1:2));yawErr=abs(atan2(sin(q(end,3)-path.goal(3)),cos(q(end,3)-path.goal(3))));
cert=struct('passed',~hit && K<=1/scene.truck.radius+1e-8 && jump<1e-8 && endErr<1e-5 && yawErr<1e-7,'collision',hit,'first',first,'motionAllowance',epsMotion,'numericAllowance',numericAllowance,'maxCurvature',K,'maxSharpness',max(abs(seg(:,3))),'curvatureJump',jump,'endError',endErr,'yawError',yawErr,'length',path.length,'sampleCount',size(q,1));
end
