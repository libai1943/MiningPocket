function [hit,first]=collidesReference(q,w,extra)
% Exact oriented rectangle versus supplied obstacle samples. Extra bounds motion between samples.
if nargin<3,extra=0;end
hit=false;first=0;if isempty(q),return;end
f=w.truck.front+w.margin+extra;b=w.truck.rear+w.margin+extra;h=w.truck.width/2+w.margin+extra;
reach=hypot(max(f,b),h); cs=cos(q(:,3));sn=sin(q(:,3));
center=q(:,1:2)+(f-b)/2*[cs sn];
support=center*w.normals'+(f+b)/2*abs([cs sn]*w.normals')+h*abs([-sn cs]*w.normals');
out=find(any(support>w.limits'+1e-9,2),1);if ~isempty(out),q=q(1:out,:);end
for k=1:size(q,1)
 if ~isempty(out)&&k==out,hit=true;first=k;return;end
 lo=max([1 1],floor((q(k,1:2)-reach-w.origin)/w.cellSize)+1);
 hi=min(w.size,floor((q(k,1:2)+reach-w.origin)/w.cellSize)+1);
 if any(lo>hi),hit=true;first=k;return;end
 [xx,yy]=ndgrid(lo(1):hi(1),lo(2):hi(2));ids=sub2ind(w.size,xx(:),yy(:));idx=vertcat(w.bins{ids});
 if isempty(idx),continue;end
 dx=w.points(idx,1)-q(k,1);dy=w.points(idx,2)-q(k,2);
 x=dx*cs(k)+dy*sn(k);y=-dx*sn(k)+dy*cs(k);
 if any(x<=f & x>=-b & abs(y)<=h),hit=true;first=k;return;end
end
end

