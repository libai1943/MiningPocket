function [result,poly,info,certificate]=reverseGate(scene,o,w)
% A finite analytic reverse witness; no optimization and no three-solver race.
result=struct('success',false,'status','NO_CONNECTION','sceneId',scene.id,'start',scene.start,'goal',scene.goal, ...
 'options',o,'sceneClass',1,'className','直接倒入','route',1,'curves',{{}},'candidates',zeros(0,5),'feasible',{{}},'best',[],'checks',0,'connectionTrace',{{}},'primaryTails',{{}});
poly=[];info=[];certificate=[];L=scene.minimumStraight;straight=cw.segment(scene.goal,L,0,o.fineStep);result.terminalStraight=straight;
if cw.collides(straight,w,o.fineStep/2),return;end
paths=[];
for rf=[1 1.4 2.4]
 p=cw.dubins(straight(end,:),scene.start,scene.truck.radius*rf);
 for j=1:numel(p),p(j).start=scene.goal;p(j).lengths=[L p(j).lengths];p(j).curvatures=[0 p(j).curvatures];p(j).length=p(j).length+L;end
 paths=[paths p];
end
if isempty(paths),return;end
[~,ix]=sort([paths.length]);paths=paths(ix);
for tail=paths
 result.checks=result.checks+1;result.primaryTails{end+1}=tail;
 if ~cw.isFree(tail,w,o),continue;end
 [pp,ii]=cw.sweptCorridor(tail,scene,o);[ok,cl,dom]=cw.polygonFree(pp,w,o.margin);if ~ok,continue;end
 b=cw.sample(tail,o.fineStep);sol=struct('head',[],'tail',tail,'forward',zeros(0,3),'reverse',flipud(b),'cusp',[],'cost',o.reverseWeight*tail.length,'forwardLength',0,'reverseLength',tail.length,'stem',L,'cuspCount',0);
 result.best=sol;result.success=true;result.status='SUCCESS';result.feasible={sol};certificate=cw.validate(scene,result);poly=pp;info=ii;info.corridorClearance=cl;info.domainClearance=dom;return;
end
end
