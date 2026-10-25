function graph=graph(scene,o,coarse)
% Layers use longitudinal progress along the anchor-to-start chord.
stem=cc.straight(scene.goal,scene.minimumStraight);a=stem.goal;b=scene.start;
d=b(1:2)-a(1:2);D=norm(d);axis=atan2(d(2),d(1));wrap=@(x)atan2(sin(x),cos(x));
qa=[0 0 wrap(a(3)-axis)];qb=[D 0 wrap(b(3)-axis)];
if nargin<3,coarse=[];end
w=cw.world(scene,cw.defaults());N=max(1,ceil(D/o.layerSpacing));
if isempty(coarse),xs=linspace(0,D,N+1);else,xs=linspace(0,D,2*N+1);end
poses=qa;layers=1;valid=true;groups=cell(1,numel(xs));groups{1}=1;oldNodes=[];
if ~isempty(coarse)
 p=coarse.best;[q,~,~]=cc.sample(p,.25);rot=[cos(axis) sin(axis);-sin(axis) cos(axis)];q(:,1:2)=(q(:,1:2)-a(1:2))*rot';q(:,3)=wrap(q(:,3)-axis);
 [ux,ix]=unique(q(:,1));uy=q(ix,2);uth=q(ix,3);
 oldNodes=coarse.graph.poses(coarse.nodePath,:);
end
for j=2:numel(xs)-1
 x=xs(j);
 if isempty(coarse),ys=o.coarseOffsets;ths=o.coarseHeadings;
 else,ys=interp1(ux,uy,x,'linear','extrap')+o.fineOffsets;ths=interp1(ux,uth,x,'linear','extrap')+o.fineHeadingOffsets;end
 [Y,T]=ndgrid(ys,ths);cand=[repmat(x,numel(Y),1),Y(:),T(:)];cand=cand(abs(cand(:,3))<deg2rad(85),:);
 if ~isempty(oldNodes),cand=[cand;oldNodes(abs(oldNodes(:,1)-x)<1e-5,:)];end
 [~,keep]=unique(round(cand*1e8)/1e8,'rows','stable');cand=cand(keep,:);
 for k=1:size(cand,1)
  p=dp.transform(struct('start',cand(k,:),'goal',cand(k,:)),a(1:2),axis);hit=cw.collides(p.start,w,0);
  poses(end+1,:)=cand(k,:);layers(end+1)=j;valid(end+1)=~hit;groups{j}(end+1)=size(poses,1);
 end
end
poses(end+1,:)=qb;layers(end+1)=numel(xs);valid(end+1)=~cw.collides(b,w,0);groups{end}=size(poses,1);
from=[];to=[];lb=[];direct=[];
% Direct connector is part of the same graph and competes by full length.
add(1,size(poses,1),true);
monotone=all(abs([qa(3) qb(3)])<deg2rad(85)) && D>6;
if monotone
 for j=1:numel(xs)-1
  for jump=1:min(2,numel(xs)-j)
   if isempty(coarse) && jump==2,continue;end
   for u=groups{j}
    if ~valid(u),continue;end
    for v=groups{j+jump}
     if ~valid(v),continue;end
     dx=poses(v,1)-poses(u,1);dy=abs(poses(v,2)-poses(u,2));
     if dy>max(6,.65*dx) || abs(poses(v,3)-poses(u,3))>deg2rad(65),continue;end
     add(u,v,false);
    end
   end
  end
 end
end
% The exact coarse incumbent remains in the refined graph.
if ~isempty(coarse)
 for k=1:size(oldNodes,1)-1
  [du,u]=min(vecnorm(poses-oldNodes(k,:),2,2));[dv,v]=min(vecnorm(poses-oldNodes(k+1,:),2,2));
  if du<1e-5 && dv<1e-5,add(u,v,(k==1 && k+1==size(oldNodes,1)));end
 end
end
graph=struct('poses',poses,'layers',layers,'valid',valid,'from',from,'to',to,'lower',lb,'direct',direct,'origin',a(1:2),'axis',axis,'stem',stem,'xs',xs,'monotone',monotone);
 function add(u,v,isDirect)
  if any(from==u & to==v),return;end
  from(end+1)=u;to(end+1)=v;lb(end+1)=norm(poses(v,1:2)-poses(u,1:2));direct(end+1)=isDirect;
 end
end
