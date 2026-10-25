function r=solve(scene,o,zero)
% Breadth-first expansion of a bounded recursion tree. A child calls cm.local.
if nargin<2,o=cm.options();end
timer=tic;o.clock=timer;if nargin<3,od=dp.options();od.refine=o.refine;zero=dp.plan(scene,od);end
w=cw.world(scene,cw.defaults());root=cm.local(scene,o,zero);
r=struct('success',root.success,'best',root.best,'root',root,'zero',zero,'nodes',{{}},'status','NO_CERTIFIED_PATH_IN_BUDGET','seconds',0,'firstSolutionSeconds',NaN,'options',o);
if root.success,r.status='SUCCESS';r.best.certificate=cm.verify(scene,r.best,o.step);assert(r.best.certificate.passed);r.seconds=toc(timer);r.firstSolutionSeconds=zero.seconds+root.firstSolutionSeconds;return;end
queue={struct('pose',scene.start,'legs',{{}},'depth',0,'parent',0)};seen=containers.Map('KeyType','char','ValueType','logical');seen(key(scene.start,0,0,0))=true;tested=0;head=1;best=[];
while head<=numel(queue) && tested<o.maxNodes && ~cm.expired(o)
 parent=queue{head};head=head+1;if parent.depth>=o.maxDepth,continue;end
 for L=o.prefixLengths
  for gear=[-1 1]
   for side=[-1 1]
    leg=cm.prefix(parent.pose,gear,side,L,scene.truck.radius,o.prefixRamp);cert=s1.check(scene,leg.path,o.step,w);
    node=struct('parent',parent.parent,'depth',parent.depth+1,'gear',gear,'side',side,'length',L,'leg',leg,'certificate',cert,'class',[],'tested',false,'reason','COLLISION','local',[],'success',false,'totalCusps',NaN);
    if cert.passed
     [qq,~,~]=cm.execution(leg,.2);pose=qq(end,:);newlegs=[parent.legs {leg}];prefix=cm.solution(newlegs,o.reverseWeight);nodeKey=key(pose,gear,prefix.cusps,parent.depth+1);
     if prefix.cusps>o.maxCusps,node.reason='CUSP_BUDGET';
     elseif isKey(seen,nodeKey),node.reason='DUPLICATE_BIN';
     elseif tested>=o.maxNodes,node.reason='NODE_BUDGET';
     else
      seen(nodeKey)=true;tested=tested+1;child=scene;child.start=pose;childOptions=o;childOptions.scaleLevels=o.childScaleLevels;local=cm.local(child,childOptions);node.local=local;node.class=local.class;node.tested=true;node.reason='NO_LOCAL_CONNECTION';
      if local.success
       sol=cm.solution([newlegs local.best.legs],o.reverseWeight);node.totalCusps=sol.cusps;
       if sol.cusps<=o.maxCusps
        sol.certificate=cm.verify(scene,sol,o.step);assert(sol.certificate.passed);if isnan(r.firstSolutionSeconds),r.firstSolutionSeconds=toc(timer);end;node.success=true;node.reason='SUCCESS';
        sol.J=evalcw.score(sol,scene.truck);if isempty(best)||sol.J<best.J-1e-9,best=sol;end
       else,node.reason='CUSP_BUDGET';end
      end
      if parent.depth+1<o.maxDepth,queue{end+1}=struct('pose',pose,'legs',{newlegs},'depth',parent.depth+1,'parent',numel(r.nodes)+1);end
     end
    end
    r.nodes{end+1}=node;
   end
  end
 end
 % Finish this parent's four-control/length set, then return the best found here.
 if ~isempty(best),break;end
end
r.success=~isempty(best);r.best=best;r.testedNodes=tested;
if r.success,r.status='SUCCESS';end
r.seconds=toc(timer);
 function k=key(pose,g,nc,depth)
  k=sprintf('%.0f,%.0f,%.0f,%d,%d,%d',round(pose(1)/o.positionBin),round(pose(2)/o.positionBin),round(atan2(sin(pose(3)),cos(pose(3)))/o.angleBin),g,nc,depth);
 end
end

