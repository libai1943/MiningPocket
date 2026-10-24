function value=cost(p,o,lastGear,lastK)
if nargin<3,lastGear=0;lastK=0;end
value=0;
for j=1:size(p,1)
 value=value+p(j,1)*(1+(o.reverseWeight-1)*(p(j,4)<0));
 if lastGear~=0,value=value+o.switchCost*(lastGear~=p(j,4));end
 value=value+o.steerCost*abs(p(j,2)-lastK)*15;lastGear=p(j,4);lastK=p(j,3);
end
end
