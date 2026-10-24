function hmap=gridHeuristic(w,target,res)
% Conventional holonomic-with-obstacles heuristic. Only raw obstacle cells;
% the full truck is always checked separately at 0.04 m on every motion.
lo=floor(min(w.points)/res)*res-res;hi=ceil(max(w.points)/res)*res+res;sz=round((hi-lo)/res)+1;
occ=false(sz);ix=round((w.points-lo)/res)+1;occ(sub2ind(sz,ix(:,1),ix(:,2)))=true;
[xx,yy]=ndgrid(lo(1):res:hi(1),lo(2):res:hi(2));inside=inpolygon(xx,yy,w.hull(:,1),w.hull(:,2));occ=occ|~inside;
t=round((target(1:2)-lo)/res)+1;t=max([1 1],min(sz,t));tid=sub2ind(sz,t(1),t(2));occ(tid)=false;d=Inf(sz);d(tid)=0;H=base.Heap;H.push(0,tid);dirs=[1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
while H.n>0
 [id,di]=H.pop();if di>d(id),continue;end;[x,y]=ind2sub(sz,id);
 for j=1:8,xy=[x y]+dirs(j,:);if any(xy<1)||any(xy>sz),continue;end;ii=sub2ind(sz,xy(1),xy(2));if occ(ii),continue;end;nd=di+res*norm(dirs(j,:));if nd<d(ii),d(ii)=nd;H.push(nd,ii);end;end
end
hmap=struct('distance',d,'origin',lo,'resolution',res,'size',sz);
end
