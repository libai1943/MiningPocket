function r=direct(scene,gear,o)
% Degenerate family: a shared CC closure and the mandatory reverse straight.
timer=tic;r=struct('solutions',{{}},'candidateCount',0,'checked',0,'seconds',0);
w=cw.world(scene,cw.defaults());tail=struct('path',cc.straight(scene.goal,scene.minimumStraight),'gear',-1,'reverseOrder',true);A=tail.path.goal;
c=s1.check(scene,tail.path,o.step,w);if ~c.passed,r.seconds=toc(timer);return;end
if gear<0,pp=cc.connect(A,scene.start,scene.truck.radius*o.directRadiusFactors,o.directRampLengths,1.8*pi);
else,pp=cc.connect(scene.start,A,scene.truck.radius*o.directRadiusFactors,o.directRampLengths,1.8*pi);end
r.candidateCount=numel(pp);
for j=1:numel(pp)
 if cm.expired(o),break;end
 r.checked=r.checked+1;c=s1.check(scene,pp{j},o.step,w);if ~c.passed,continue;end
 leg=struct('path',pp{j},'gear',gear,'reverseOrder',gear<0);
 sol=cm.solution({leg,tail},o.reverseWeight);sol.certificate=cm.verify(scene,sol,o.step);
 if ~sol.certificate.passed,continue;end
 [J,e]=evalcw.score(sol,scene.truck);if ~e.valid,continue;end
 sol.J=J;r.solutions{end+1}=sol;
 if numel(r.solutions)>=o.feasibleLimit,break;end
end
r.seconds=toc(timer);
end
