function result=regionalWide(scene,o,w,result)
% Broad, deterministic pose coverage. Heading priorities differ by scene geometry.

g=scene.goal;g(1:2)=g(1:2)+scene.minimumStraight*[cos(g(3)) sin(g(3))];
[path,info]=cw.guide(w.points,[],g(1:2),scene.start(1:2),scene.truck.width/2+o.margin,.75);
result.guide=path;result.guideStatus=info.status;if isempty(path),return;end
s=[0;cumsum(vecnorm(diff(path),2,2))];[s,ix]=unique(s);path=path(ix,:);seeds=interp1(s,path,unique([0:5:s(end),s(end)]));
xy=[];for i=1:size(seeds,1),[dx,dy]=meshgrid(-12:4:12);xy=[xy;seeds(i,:)+[dx(:) dy(:)]];end %#ok<AGROW>
xy=unique(round(xy/2)*2,'rows');[~,ix]=sort(vecnorm(xy-scene.goal(1:2),2,2)+vecnorm(xy-scene.start(1:2),2,2));xy=xy(ix,:);
[cls,~]=cw.headingSector(scene);a=scene.goal(3);b=scene.start(3);headingDelta=atan2(sin(b-a),cos(b-a));
if cls==1,priority=a+deg2rad([0 30 -30 60 -60 90 -90]);
elseif cls==2,priority=a+headingDelta/2+deg2rad([0 15 -15 30 -30 60 -60]);
else,priority=a+deg2rad([90 -90 75 -75 105 -105 60 -60 120 -120]);end
yaws=unique(round(atan2(sin([priority a+(-pi:pi/12:pi)]),cos([priority a+(-pi:pi/12:pi)]))*1e8)/1e8,'stable');
sweep=o.step/2*(1+hypot(scene.truck.front,scene.truck.width/2)/scene.truck.radius);
% Reusable analytic approach pieces allow a bend in the entry corridor.
prefixes={};fractions=[.2 .35 .5 .65 .8];
for u=fractions
 dist=u*s(end);pos=interp1(s,path,dist);p1=interp1(s,path,max(0,dist-2));p2=interp1(s,path,min(s(end),dist+2));yaw=atan2(p1(2)-p2(2),p1(1)-p2(1));
 for da=deg2rad([0 -10 10])
  q=[pos yaw+da];if cw.collides(q,w,sweep),continue;end
  pp=cw.dubins(scene.start,q,scene.truck.radius);
  for j=1:numel(pp)
   if pp(j).length>2*norm(q(1:2)-scene.start(1:2))+20,continue;end
   if cw.isFree(pp(j),w,o),prefixes{end+1}=pp(j);break;end %#ok<AGROW>
  end
 end
end
result.regionalPoints=xy;result.regionalTail={};result.regionalTried=0;result.prefixCount=numel(prefixes);best=inf;
for yaw=yaws
 for i=1:size(xy,1)
  q=[xy(i,:) yaw];if cw.collides(q,w,sweep),continue;end
  result.regionalTried=result.regionalTried+1;if result.regionalTried>o.maxCandidates,return;end
  good=[];
  for rf=[1 1.3]
   tails=cw.dubins(g,q,scene.truck.radius*rf);
   for j=1:numel(tails)
    e=tails(j);tail=e;tail.start=scene.goal;tail.lengths=[scene.minimumStraight e.lengths];tail.curvatures=[0 e.curvatures];tail.length=scene.minimumStraight+e.length;
    if tail.length>norm(q(1:2)-scene.goal(1:2))+pi*2*scene.truck.radius,continue;end
    if o.reverseWeight*tail.length+norm(scene.start(1:2)-q(1:2))>=best,continue;end
    result.checks=result.checks+1;if ~cw.isFree(tail,w,o),continue;end
    if isempty(good)||tail.length<good.length,good=tail;end
    break;
   end
  end
  if isempty(good),continue;end
  if numel(result.regionalTail)<180,result.regionalTail{end+1}=cw.sample(good,o.step);end
  result.candidates(end+1,:)=[q 200 0];headGood=[];
  heads=cw.dubins(scene.start,q,scene.truck.radius);
  for j=1:numel(heads)
   head=heads(j);cost=head.length+o.reverseWeight*good.length;if cost>=best,continue;end
   result.checks=result.checks+1;if ~cw.isFree(head,w,o),continue;end
   headGood=head;break;
  end
  if isempty(headGood)
   distances=inf(1,numel(prefixes));
   for j=1:numel(prefixes),pp=cw.sample(prefixes{j},5);distances(j)=norm(pp(end,1:2)-q(1:2));end
   [~,preOrder]=sort(distances);
   for pre=preOrder(1:min(3,numel(preOrder)))
    prefix=prefixes{pre};pq=cw.sample(prefix,2);heads=cw.dubins(pq(end,:),q,scene.truck.radius);
    for j=1:numel(heads)
     h=heads(j);cost=prefix.length+h.length+o.reverseWeight*good.length;if cost>=best,continue;end
     if prefix.length+h.length>2.5*norm(scene.start(1:2)-q(1:2))+30,continue;end
     result.checks=result.checks+1;if ~cw.isFree(h,w,o),continue;end
     h.start=scene.start;h.lengths=[prefix.lengths h.lengths];h.curvatures=[prefix.curvatures h.curvatures];h.length=prefix.length+h.length;
     if isempty(headGood)||h.length<headGood.length,headGood=h;end
     break;
    end
   end
  end
  if isempty(headGood),continue;end
  f=cw.sample(headGood,o.fineStep);bq=cw.sample(good,o.fineStep);fine=o.fineStep/2*(1+hypot(scene.truck.front,scene.truck.width/2)/scene.truck.radius);
  if cw.collides(f,w,fine)||cw.collides(bq,w,fine),continue;end
  cost=headGood.length+o.reverseWeight*good.length;
  sol=struct('head',headGood,'tail',good,'forward',f,'reverse',flipud(bq),'cusp',q,'cost',cost,'forwardLength',headGood.length,'reverseLength',good.length,'stem',scene.minimumStraight,'cuspCount',1);
  best=cost;result.best=sol;result.route=3;result.feasible{end+1}=sol;
 end
 if ~isempty(result.best),return;end % Complete this heading layer after the first solution.
end
end
