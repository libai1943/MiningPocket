function [hit,first]=collides(q,w,extra)
% Batched exact rectangle/point collision, preserving earliest collision index.
if nargin<3,extra=0;end
hit=false;first=0;if isempty(q),return;end
f=w.truck.front+w.margin+extra;b=w.truck.rear+w.margin+extra;h=w.truck.width/2+w.margin+extra;
reach=hypot(max(f,b),h);cs=cos(q(:,3));sn=sin(q(:,3));
center=q(:,1:2)+(f-b)/2*[cs sn];support=center*w.normals'+(f+b)/2*abs([cs sn]*w.normals')+h*abs([-sn cs]*w.normals');
out=find(any(support>w.limits'+1e-9,2),1);last=size(q,1);if ~isempty(out),last=out-1;end
for k=1:24:last
 ix=k:min(k+23,last);qq=q(ix,:);
 lo=max([1 1],floor((min(qq(:,1:2))-reach-w.origin)/w.cellSize)+1);
 hi=min(w.size,floor((max(qq(:,1:2))+reach-w.origin)/w.cellSize)+1);
 [xx,yy]=ndgrid(lo(1):hi(1),lo(2):hi(2));ids=sub2ind(w.size,xx(:),yy(:));idx=vertcat(w.bins{ids});
 if isempty(idx),continue;end
 dx=w.points(idx,1)-qq(:,1)';dy=w.points(idx,2)-qq(:,2)';
 x=dx.*cs(ix)'+dy.*sn(ix)';y=-dx.*sn(ix)'+dy.*cs(ix)';
 j=find(any(x<=f & x>=-b & abs(y)<=h,1),1);
 if ~isempty(j),hit=true;first=ix(j);return;end
end
if ~isempty(out),hit=true;first=out;end
end
