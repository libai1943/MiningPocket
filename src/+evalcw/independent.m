function a=independent(scene,sol)
% Independent point/rectangle arithmetic: no spatial bins or planner collision call.
% It checks the fixed benchmark obstacle representation and hull, with a
% conservative between-sample motion bound on every 0.02 m interval.
p=scene.boundary;t=scene.truck;
if ~isempty(scene.parkedTruck)
 [xx,yy]=meshgrid(-t.rear:.25:t.front,-t.width/2:.25:t.width/2);q0=scene.parkedTruck;rot=[cos(q0(3)) -sin(q0(3));sin(q0(3)) cos(q0(3))];p=[p;[xx(:) yy(:)]*rot'+q0(1:2)];
end
h=scene.boundary(convhull(scene.boundary(:,1),scene.boundary(:,2)),:);edge=diff(h);normals=[edge(:,2) -edge(:,1)];normals=normals./vecnorm(normals,2,2);limits=sum(normals.*h(1:end-1,:),2);
minimum=Inf;count=0;previous=scene.start;err=[0 0];passed=true;
for j=1:numel(sol.legs)
 [q,s,k]=cm.execution(sol.legs{j},.02);err=max(err,[norm(q(1,1:2)-previous(1:2)),abs(atan2(sin(q(1,3)-previous(3)),cos(q(1,3)-previous(3))))]);previous=q(end,:);
 rho=hypot(t.front+.5,t.width/2+.5);delta=max(diff(s))/2*(1+rho*max(abs(k)))+1e-5;
 f=t.front+.5+delta;b=t.rear+.5+delta;hw=t.width/2+.5+delta;reach=hypot(max(f,b),hw);cs=cos(q(:,3));sn=sin(q(:,3));center=q(:,1:2)+(f-b)/2*[cs sn];
 support=center*normals'+(f+b)/2*abs([cs sn]*normals')+hw*abs([-sn cs]*normals');gap=limits'-support;minimum=min(minimum,min(gap,[],'all'));passed=passed&&min(gap,[],'all')>=-1e-9&&max(abs(k))<=1/t.radius+1e-8;
 for first=1:32:size(q,1)
  qq=q(first:min(first+31,size(q,1)),:);lo=min(qq(:,1:2),[],1)-reach;hi=max(qq(:,1:2),[],1)+reach;keep=p(:,1)>=lo(1)&p(:,1)<=hi(1)&p(:,2)>=lo(2)&p(:,2)<=hi(2);pp=p(keep,:);if isempty(pp),continue;end
  dx=pp(:,1)-qq(:,1)';dy=pp(:,2)-qq(:,2)';xx=dx.*cos(qq(:,3)')+dy.*sin(qq(:,3)');yy=-dx.*sin(qq(:,3)')+dy.*cos(qq(:,3)');gap=max(max(xx-f,-b-xx),abs(yy)-hw);minimum=min(minimum,min(gap,[],'all'));passed=passed&&all(gap(:)>0);
 end
 count=count+size(q,1);
end
err=max(err,[norm(previous(1:2)-scene.goal(1:2)),abs(atan2(sin(previous(3)-scene.goal(3)),cos(previous(3)-scene.goal(3))))]);pieces=evalcw.pieces(sol);tail=0;
for j=size(pieces,1):-1:1,if pieces(j,4)==-1&&max(abs(pieces(j,2:3)))<1e-9,tail=tail+pieces(j,1);else,break;end;end
passed=passed&&err(1)<1e-5&&err(2)<1e-7&&tail>=scene.minimumStraight-1e-6;
a=struct('passed',passed,'samples',count,'minimumResidual',minimum,'poseError',err,'terminalStraight',tail,'step',.02,'margin',.5,'usesPlannerCollisionFunction',false);
end
