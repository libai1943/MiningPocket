function cert=verify(scene,sol,step)
if nargin<3,step=.04;end
w=cw.world(scene,cw.defaults());certs=cell(1,numel(sol.legs));joinXY=0;joinYaw=0;joinK=0;previous=[];
for j=1:numel(sol.legs)
 leg=sol.legs{j};certs{j}=s1.check(scene,leg.path,step,w);[q,~,k]=cm.execution(leg,step);
 if j==1,startXY=norm(q(1,1:2)-scene.start(1:2));startYaw=abs(atan2(sin(q(1,3)-scene.start(3)),cos(q(1,3)-scene.start(3))));
 else
  joinXY=max(joinXY,norm(previous(1:2)-q(1,1:2)));joinYaw=max(joinYaw,abs(atan2(sin(previous(3)-q(1,3)),cos(previous(3)-q(1,3)))));joinK=max(joinK,abs(previous(4)-k(1)));
 end
 previous=[q(end,:) k(end)];
end
endXY=norm(previous(1:2)-scene.goal(1:2));endYaw=abs(atan2(sin(previous(3)-scene.goal(3)),cos(previous(3)-scene.goal(3))));
lastLeg=sol.legs{end};[~,lastS,lastK]=cm.execution(lastLeg,step);
need=lastS>=lastLeg.path.length-scene.minimumStraight-1e-7;terminal=lastLeg.gear==-1 && lastLeg.path.length>=scene.minimumStraight-1e-7 && max(abs(lastK(need)))<1e-8;
cert=struct('passed',all(cellfun(@(x)x.passed,certs)) && max([joinXY startXY endXY])<1e-5 && max([joinYaw startYaw endYaw])<1e-7 && joinK<1e-8 && terminal,'legs',{certs},'joinXY',joinXY,'joinYaw',joinYaw,'joinCurvature',joinK,'startError',startXY,'endError',endXY,'terminalStraight',terminal,'cusps',sol.cusps);
end
