function [free,clearance,domain]=polygonFree(poly,w,margin)
% Filled convex corridor, not merely its boundary or its diagonals.
h=convhull(poly(:,1),poly(:,2));poly=poly(h(1:end-1),:);q=poly([2:end 1],:);e=q-poly;
n=[e(:,2) -e(:,1)];n=n./vecnorm(n,2,2);b=sum(n.*poly,2);
inside=all(w.points*n'<=b'+1e-10,2);clearance=inf;
if any(inside),clearance=0;else
 for j=1:size(poly,1)
  d=w.points-poly(j,:);u=max(0,min(1,(d*e(j,:)')/sum(e(j,:).^2)));dd=d-u.*e(j,:);clearance=min(clearance,sqrt(min(sum(dd.^2,2))));
 end
end
domain=min(w.limits'-poly*w.normals',[],'all');free=clearance>=margin+1e-9 && domain>=margin+1e-9;
end
