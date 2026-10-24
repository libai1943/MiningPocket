function cert=validate(scene,sol,step)
if nargin<3,step=.02;end
w=cw.world(scene,cw.defaults());previous=scene.start;errors=[0 0];passed=true;count=0;
for j=1:numel(sol.legs)
 leg=sol.legs{j};[q,s,k]=cm.execution(leg,step);errors=max(errors,[norm(q(1,1:2)-previous(1:2)),abs(atan2(sin(q(1,3)-previous(3)),cos(q(1,3)-previous(3))))]);
 if ~base.free(q,w,max(abs(k)),max(diff(s)))||max(abs(k))>1/scene.truck.radius+1e-8,passed=false;end
 previous=q(end,:);count=count+size(q,1);
end
errors=max(errors,[norm(previous(1:2)-scene.goal(1:2)),abs(atan2(sin(previous(3)-scene.goal(3)),cos(previous(3)-scene.goal(3))))]);
p=evalcw.pieces(sol);tail=0;for j=size(p,1):-1:1,if p(j,4)==-1&&max(abs(p(j,2:3)))<1e-9,tail=tail+p(j,1);else,break;end;end
cert=struct('passed',passed&&errors(1)<1e-5&&errors(2)<1e-7&&tail>=scene.minimumStraight-1e-6,'endpointError',errors,'terminalStraight',tail,'samples',count);
end
