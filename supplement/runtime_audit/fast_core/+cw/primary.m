function result=primary(scene,o,gate)
% Three different constructions selected before generating candidate trajectories.
% Each uses the same exact circular-arc connector and swept-body validator.
if gate.class==1 && isfield(gate,'witness') && ~isempty(gate.witness)
 result=gate.witness;return;
end
if gate.class==2
 local=o;local.mode='adaptive';local.skipDirect=true;local.skipRegional=true;
 result=cw.corePlan(scene,local);result.options=o;return;
end
w=cw.world(scene,o);t=scene.truck;L=scene.minimumStraight;
result=struct('success',false,'status','NO_CONNECTION','sceneId',scene.id,'start',scene.start,'goal',scene.goal, ...
 'options',o,'sceneClass',gate.class,'className',gate.name,'route',0,'curves',{{}},'candidates',zeros(0,5),'feasible',{{}},'best',[],'checks',0,'connectionTrace',{{}},'startCurves',{{}},'primaryTails',{{}});
if cw.collides(scene.start,w,0),result.status='START_COLLISION';return;end
if cw.collides(scene.goal,w,0),result.status='GOAL_COLLISION';return;end
straight=cw.segment(scene.goal,L,0,o.fineStep);result.terminalStraight=straight;
if cw.collides(straight,w,o.fineStep/2),result.status='TERMINAL_STRAIGHT_BLOCKED';return;end
a=straight(end,:);best=inf;extra=o.fineStep/2*(1+hypot(t.front,t.width/2)/t.radius);
if gate.class==1
 % Complete reverse connectors: no cusp is imposed on an aligned arrival.
 tryConnection([],scene.start);
else
 % Opposed arrivals: enumerate forward steering arcs FIRST; their endpoints are cusp poses.
 % This is a distinct root and enumeration variable from the lateral goal-out construction.
 for k=[0 -1/t.radius 1/t.radius -1/(1.4*t.radius) 1/(1.4*t.radius)]
  arc=cw.segment(scene.start,48,k,o.fineStep);[hit,idx]=cw.collides(arc,w,extra);
  if ~hit,idx=size(arc,1)+1;end
  safe=max(0,(min(idx-1,size(arc,1))-1)*48/(size(arc,1)-1)-o.backoff);
  result.startCurves{end+1}=struct('q',arc(1:min(idx,end),:),'hit',hit,'safeLength',safe,'curvature',k);
  for d=unique([4:4:safe safe])
   if d<1,continue;end
   head=struct('start',scene.start,'lengths',d,'curvatures',k,'length',d,'word',0);
   q=cw.sample(head,o.fineStep);pose=q(end,:);result.candidates(end+1,:)=[pose 300 d];
   tryConnection(head,pose);
  end
 end
end
if ~isempty(result.best),result.success=true;result.status='SUCCESS';result.route=gate.class;end
 function tryConnection(head,pose)
  for rf=[1 1.4 2.4]
   paths=cw.dubins(a,pose,t.radius*rf);
   for jj=1:numel(paths)
    tail=paths(jj);tail.start=scene.goal;tail.lengths=[L tail.lengths];tail.curvatures=[0 tail.curvatures];tail.length=L+tail.length;
    fl=0;if ~isempty(head),fl=head.length;end
    cost=fl+o.reverseWeight*tail.length;if cost>=best,continue;end
    result.checks=result.checks+1;result.primaryTails{end+1}=tail;
    if ~cw.isFree(tail,w,o),continue;end
    b=cw.sample(tail,o.fineStep);if cw.collides(b,w,extra),continue;end
    if isempty(head),f=zeros(0,3);cusp=[];else,f=cw.sample(head,o.fineStep);cusp=f(end,:);if cw.collides(f,w,extra),continue;end;end
    sol=struct('head',head,'tail',tail,'forward',f,'reverse',flipud(b),'cusp',cusp,'cost',cost,'forwardLength',fl,'reverseLength',tail.length,'stem',L,'cuspCount',~isempty(head));
    result.best=sol;result.feasible{end+1}=sol;best=cost;
   end
  end
 end
end
