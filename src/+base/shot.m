function [ok,p,bestCost]=shot(start,goal,w,o,kind,lastGear,lastK)
if nargin<6,lastGear=0;lastK=0;end
[pSet,costs]=base.connections(start,goal,w.truck.radius,o,kind);ok=false;p=[];bestCost=Inf;
for j=1:numel(pSet),costs(j)=base.cost(pSet{j},o,lastGear,lastK);end
[~,order]=sort(costs);
for j=order
 v=pSet{j};q=start;valid=true;
 for k=1:size(v,1)
  qq=base.primitive(q,v(k,1),v(k,2),v(k,4),o.step);
  if ~base.free(qq,w,v(k,2),v(k,1)/(size(qq,1)-1)),valid=false;break;end;q=qq(end,:);
 end
 if valid && norm(q(1:2)-goal(1:2))<1e-5&&abs(atan2(sin(q(3)-goal(3)),cos(q(3)-goal(3))))<1e-7,ok=true;p=v;bestCost=costs(j);return;end
end
end
