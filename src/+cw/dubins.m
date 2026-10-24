function paths=dubins(a,b,r)
% Six analytic forward connectors. Closed-form equations, no numerical optimisation.
delta=b(1:2)-a(1:2);d=norm(delta)/r;phi=atan2(delta(2),delta(1));al=mod(a(3)-phi,2*pi);be=mod(b(3)-phi,2*pi);
sa=sin(al);sb=sin(be);ca=cos(al);cb=cos(be);cab=cos(al-be);P=nan(6,3);
v=2+d*d-2*cab+2*d*(sa-sb);
if v>=-1e-10,t=atan2(cb-ca,d+sa-sb);P(1,:)=[wrap(-al+t),sqrt(max(0,v)),wrap(be-t)];end
v=2+d*d-2*cab+2*d*(sb-sa);
if v>=-1e-10,t=atan2(ca-cb,d-sa+sb);P(2,:)=[wrap(al-t),sqrt(max(0,v)),wrap(-be+t)];end
v=d*d-2+2*cab-2*d*(sa+sb);
if v>=-1e-10,p=sqrt(max(0,v));t=atan2(ca+cb,d-sa-sb)-atan2(2,p);P(3,:)=[wrap(al-t),p,wrap(be-t)];end
v=d*d-2+2*cab+2*d*(sa+sb);
if v>=-1e-10,p=sqrt(max(0,v));t=atan2(-ca-cb,d+sa+sb)-atan2(-2,p);P(4,:)=[wrap(-al+t),p,wrap(-be+t)];end
v=(6-d*d+2*cab+2*d*(sa-sb))/8;
if abs(v)<=1,p=2*pi-acos(v);t=wrap(al-atan2(ca-cb,d-sa+sb)+p/2);P(5,:)=[t,p,wrap(al-be-t+p)];end
v=(6-d*d+2*cab+2*d*(-sa+sb))/8;
if abs(v)<=1,p=2*pi-acos(v);t=wrap(-al+atan2(-ca+cb,d+sa-sb)+p/2);P(6,:)=[t,p,wrap(be-al-t+p)];end
K=[1 0 1;-1 0 -1;-1 0 1;1 0 -1;-1 1 -1;1 -1 1];paths=struct('start',{},'lengths',{},'curvatures',{},'length',{},'word',{});
for j=1:6
 if any(isnan(P(j,:))),continue;end
 z=struct('start',a,'lengths',P(j,:)*r,'curvatures',K(j,:)/r,'length',sum(P(j,:))*r,'word',j);
 paths(end+1)=z; %#ok<AGROW>
end
if ~isempty(paths),[~,ii]=sort([paths.length]);paths=paths(ii);end
end
function a=wrap(a)
a=mod(a,2*pi);if min(a,2*pi-a)<1e-10,a=0;end
end
