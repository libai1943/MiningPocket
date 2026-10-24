function [poly,info]=sweptCorridor(path,scene,o)
% Circumscribed convex polygon: every continuous swept body point lies inside.
d=scene.start(1:2)-scene.goal(1:2);if norm(d)<1e-9,d=[cos(scene.goal(3)) sin(scene.goal(3))];end;u=d/norm(d);v=[-u(2) u(1)];
a=atan2(u(2),u(1))+(0:o.gateNormals-1)'*2*pi/o.gateNormals;n=[cos(a) sin(a)];b=cw.sweptSupport(path,scene.truck,n)+o.gateBuffer;
poly=zeros(size(n));for j=1:size(n,1),k=mod(j,size(n,1))+1;poly(j,:)=(n([j k],:)\b([j k]))';end
h=convhull(poly(:,1),poly(:,2));poly=poly(h(1:end-1),:);
base=[cw.vehicle(scene.start,scene.truck);cw.vehicle(scene.goal,scene.truck)];sv=cw.sweptSupport(path,scene.truck,[v;-v]);bv=max(base*[v;-v]',[],1)';
su=cw.sweptSupport(path,scene.truck,[u;-u]);bu=max(base*[u;-u]',[],1)';
info=struct('axis',u,'normal',v,'leftExpansion',max(0,sv(1)-bv(1))+o.gateBuffer,'rightExpansion',max(0,sv(2)-bv(2))+o.gateBuffer,'longitudinalExpansion',max(0,su-bu), ...
 'supportNormals',n,'supportLimits',b,'extraBuffer',o.gateBuffer,'safetyMargin',o.margin,'exactContinuousSupport',true);
end
