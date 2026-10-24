function result=domainAttempt(scene,o,result)
% Extra diagnostic: lower corridor pockets beyond the original goal-start guide.
w=cw.world(scene,o);t=scene.truck;timer=tic;records={};n=0;best=inf;
if result.success,best=result.best.cost;end
lo=min(scene.boundary);hi=max(scene.boundary);poses=[];
for y=lo(2)+15:15:min(scene.goal(2)-10,scene.start(2)-10)
 for x=lo(1)+8:8:hi(1)-8
  for a=scene.goal(3)+pi+deg2rad([-30 0 30])
   q=[x y a];if ~cw.collides(q,w,.1),poses(end+1,:)=q;end
  end
 end
end
% Goal-side straight prefixes move the first turn past an internal wall tip.
for L=scene.minimumStraight:4:90
 line=cw.segment(scene.goal,L,0,o.fineStep);if cw.collides(line,w,.05),break;end
 for i=1:size(poses,1)
  edges=cw.dubins(line(end,:),poses(i,:),t.radius);
  for j=1:numel(edges)
   e=edges(j);if ~cw.isFree(e,w,o),continue;end
   prefix=e;prefix.start=scene.goal;prefix.lengths=[L e.lengths];prefix.curvatures=[0 e.curvatures];prefix.length=L+e.length;
   records{end+1}=cw.sample(prefix,.5);
   next=cw.dubins(poses(i,:),scene.start,t.radius);
   for k=1:numel(next)
    h=next(k);n=n+1;len=prefix.length+h.length;if o.reverseWeight*len>=best,continue;end
    if ~cw.isFree(h,w,o),continue;end
    tail=prefix;tail.lengths=[prefix.lengths h.lengths];tail.curvatures=[prefix.curvatures h.curvatures];tail.length=len;
    q=cw.sample(tail,o.fineStep);if cw.collides(q,w,o.fineStep/2*(1+hypot(t.front,t.width/2)/t.radius)),continue;end
    sol=struct('head',[],'tail',tail,'forward',zeros(0,3),'reverse',flipud(q),'cusp',[],'cost',o.reverseWeight*len,'forwardLength',0,'reverseLength',len,'stem',L,'cuspCount',0);
    result.best=sol;result.feasible{end+1}=sol;result.success=true;result.status='SUCCESS';result.route=4;best=sol.cost;
   end
   break;
  end
 end
end
result.domainTrace=struct('poses',poses,'prefixes',{records},'connections',n,'seconds',toc(timer));
end
