function sol=build(start,p)
legs={};q=start;
for j=1:size(p,1)
 if p(j,1)<1e-9,continue;end
 L=p(j,1);k=p(j,2);g=p(j,4);tangent=q;tangent(3)=tangent(3)+(g<0)*pi;
 goal=base.primitive(q,L,k,g,L);goal=goal(end,:);pGoal=goal;pGoal(3)=pGoal(3)+(g<0)*pi;
 path=struct('start',tangent,'goal',pGoal,'segments',[L,g*k,0],'length',L,'bodyOffset',(g<0)*pi);
 legs{end+1}=struct('path',path,'gear',g,'reverseOrder',false);q=goal; %#ok<AGROW>
end
sol=cm.solution(legs);sol.endPose=q;
end
