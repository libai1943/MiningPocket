function result=corePlan(scene,o)
if nargin<2,o=cw.defaults();end
timer=tic;w=cw.world(scene,o);t=scene.truck;
[sceneClass,className]=cw.headingSector(scene);
sweep=o.step/2*(1+hypot(t.front,t.width/2)/t.radius);
result=struct('success',false,'status','NO_CONNECTION','sceneId',scene.id,'start',scene.start,'goal',scene.goal, ...
 'options',o,'sceneClass',sceneClass,'className',className,'route',0,'curves',{{}},'candidates',zeros(0,5),'feasible',{{}},'best',[],'checks',0,'connectionTrace',{{}});
if cw.collides(scene.start,w,0),result.status='START_COLLISION';result.seconds=toc(timer);return;end
if cw.collides(scene.goal,w,0),result.status='GOAL_COLLISION';result.seconds=toc(timer);return;end
stemLengths=scene.minimumStraight:o.stemStep:o.maxStem;bestCost=inf;
% A valid zero-cusp reverse solution belongs to the same representation.
straight=cw.segment(scene.goal,scene.minimumStraight,0,o.step);
if cw.collides(cw.segment(scene.goal,scene.minimumStraight,0,o.fineStep),w,o.fineStep/2),result.status='TERMINAL_STRAIGHT_BLOCKED';result.seconds=toc(timer);return;end
result.terminalStraight=straight;
ends=cw.dubins(straight(end,:),scene.start,t.radius);
if isfield(o,'skipDirect')&&o.skipDirect,ends=ends([]);end
for jj=1:numel(ends)
 e=ends(jj);tail=e;tail.start=scene.goal;tail.lengths=[scene.minimumStraight e.lengths];
 tail.curvatures=[0 e.curvatures];tail.length=scene.minimumStraight+e.length;
 q=cw.sample(tail,o.step);result.checks=result.checks+1;
 if ~cw.isFree(tail,w,o),continue;end
 fine=cw.sample(tail,o.fineStep);
 if cw.collides(fine,w,o.fineStep/2*(1+hypot(t.front,t.width/2)/t.radius)),continue;end
 cost=o.reverseWeight*tail.length;
 if cost<bestCost
  bestCost=cost;result.best=struct('head',[],'tail',tail,'forward',zeros(0,3),'reverse',flipud(fine), ...
   'cusp',[],'cost',cost,'forwardLength',0,'reverseLength',tail.length,'stem',scene.minimumStraight,'cuspCount',0);
  result.route=1;result.feasible{end+1}=result.best;
 end
end

for si=1:numel(stemLengths)
 L=stemLengths(si);straight=cw.segment(scene.goal,L,0,o.step);
 if cw.collides(cw.segment(scene.goal,L,0,o.fineStep),w,o.fineStep/2),break;end
 if si==1,result.terminalStraight=straight;end
 newFound=false;
 for factor=o.radiusFactors
  r=t.radius*factor;
  for sign=[-1 1]
   arc=cw.segment(straight(end,:),r*o.maxTurn,sign/r,o.fineStep);
   [hit,k]=cw.collides(arc,w,o.fineStep/2*(1+hypot(t.front,t.width/2)/r));if ~hit,k=size(arc,1)+1;end
   safe=max(1,k-1);arcLength=(safe-1)*r*o.maxTurn/(size(arc,1)-1);
   limit=max(0,arcLength-o.backoff);
   record=struct('stem',L,'radius',r,'side',sign,'q',arc(1:min(k,end),:),'hit',hit,'safeLength',limit);
   result.curves{end+1}=record;
   if limit<r*deg2rad(8),continue;end
   angles=unique([deg2rad(8):o.angleStep:limit/r,limit/r]);
    % Keep the same coverage, but test each scene's useful heading sector first.
    if sceneClass==1,preferred=0;elseif sceneClass==2,preferred=pi/4;else,preferred=pi/2;end
    [~,order]=sort(abs(angles-preferred));angles=angles(order);
   for angle=angles
    tail=struct('start',scene.goal,'lengths',[L r*angle],'curvatures',[0 sign/r],'length',L+r*angle,'word',0);
    tailQ=cw.sample(tail,o.step);cusp=tailQ(end,:);
    result.candidates(end+1,:)=[cusp si angle];
    connections=cw.dubins(scene.start,cusp,t.radius);
    for j=1:numel(connections)
     head=connections(j);cost=head.length+o.reverseWeight*tail.length;
     if cost>=bestCost || head.length>norm(scene.start(1:2)-cusp(1:2))+pi*3*t.radius,continue;end
     result.checks=result.checks+1;headQ=cw.sample(head,o.step);
     if isfield(o,'detailedTrace')&&o.detailedTrace,result.connectionTrace{end+1}=head;end
     if ~cw.isFree(head,w,o),continue;end
     % Independent finer validation before accepting a winner.
     fineExtra=o.fineStep/2*(1+hypot(t.front,t.width/2)/t.radius);
     fq=cw.sample(head,o.fineStep);bq=cw.sample(tail,o.fineStep);
     if cw.collides(fq,w,fineExtra)||cw.collides(bq,w,fineExtra),continue;end
     sol=struct('head',head,'tail',tail,'forward',fq,'reverse',flipud(bq),'cusp',cusp,'cost',cost, ...
      'forwardLength',head.length,'reverseLength',tail.length,'stem',L,'cuspCount',1);
     result.feasible{end+1}=sol;result.best=sol;bestCost=cost;newFound=true;
    end
   end
  end
 end
 if newFound && si==1,result.route=1;end
 if newFound && result.route==0,result.route=2;end
 % Local success is a complete branch; otherwise grow the straight to seek room.
 if result.route==1 && strcmp(o.mode,'adaptive'),break;end
 if si>=2 && ~isempty(result.best) && strcmp(o.mode,'adaptive'),break;end
 if strcmp(o.mode,'local_only'),break;end
end
if isempty(result.best) && ~strcmp(o.mode,'local_only') && ~(isfield(o,'skipRegional')&&o.skipRegional)
 result=cw.regionalWide(scene,o,w,result);
end
if ~isempty(result.best),result.success=true;result.status='SUCCESS';end
result.seconds=toc(timer);
end
