function r=one(scene,kind)
% One gear change, two zero-end-curvature paths sharing a pose.
% kind=2: goal-out fan; kind=3: start-forward fan.
timer=tic;label=oldcc.classify(scene);r=struct('success',false,'status','BUDGET_NO_CONNECTION','class',label,'kind',kind,'fans',{{}},'poses',zeros(0,3),'tried',0,'connections',{{}},'feasible',{{}},'best',[],'seconds',0);
R=scene.truck.radius;best=inf;
if kind==2,stemLengths=scene.minimumStraight+[0 8 16];else,stemLengths=[0 8];end
for stemLength=stemLengths
 if kind==2,rootPose=scene.goal;else,rootPose=scene.start;end
 stem=oldcc.straight(rootPose,stemLength);anchor=stem.goal;
 for radius=R*[1 1.4]
  for ramp=[4 8]
   for side=[-1 1]
    for angle=deg2rad(15:15:150)
     turn=struct('start',anchor,'goal',[0 0 0],'segments',oldcc.turnSegments(side*angle,radius,ramp),'length',0);
     turn.length=sum(turn.segments(:,1));q=oldcc.sample(turn,.3);turn.goal=q(end,:);fan=oldcc.join(stem,turn);fan.goal=turn.goal;
     prefixCert=oldcc.check(scene,fan,.15);
     r.fans{end+1}=struct('path',fan,'certificate',prefixCert,'side',side,'angle',angle);
     if ~prefixCert.passed,continue;end
     pose=fan.goal;r.poses(end+1,:)=pose;
     if kind==2,connections=oldcc.connect(scene.start,pose,R*[1 1.4],4,1.8*pi);
     else
      last=oldcc.straight(scene.goal,scene.minimumStraight);
      connections=oldcc.connect(last.goal,pose,R*[1 1.4],4,1.8*pi);
     end
     for j=1:numel(connections)
      if kind==2,head=connections{j};tail=fan;else,head=fan;tail=oldcc.join(last,connections{j});end
      cost=head.length+1.35*tail.length;
      r.tried=r.tried+1;
      if kind==2,p= head;else,p=tail;end
      cert=oldcc.check(scene,p,.15);r.connections{end+1}=struct('path',p,'certificate',cert,'cusp',pose);
      if ~cert.passed,continue;end
      ch=oldcc.check(scene,head,.04);ct=oldcc.check(scene,tail,.04);
      if ~ch.passed||~ct.passed,continue;end
      sol=struct('head',head,'tail',tail,'cusp',pose,'cost',cost,'length',head.length+tail.length,'cuspCount',1,'headCertificate',ch,'tailCertificate',ct);
      r.feasible{end+1}=sol;
      if cost<best,r.best=sol;best=cost;end
     end
    end
   end
  end
 end
 % Complete all angle/radius families for this anchor, then stop if successful.
 if ~isempty(r.best),break;end
end
if ~isempty(r.best),r.success=true;r.status='SUCCESS';end
r.seconds=toc(timer);
end
