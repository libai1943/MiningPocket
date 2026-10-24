function drawGate(ax,scene,result,layer)
if nargin<4,layer='all';end
cb.draw(ax,scene,result,1);g=result.gate;
if any(strcmp(layer,{'all','links'}))
 for k=1:16
  p=g.cornerLinks(k,:);ij=[floor((k-1)/4)+1 mod(k-1,4)+1];col=[.25 .66 .52];if ~g.linkFree(ij(1),ij(2)),col=[.84 .32 .24];end
  plot(ax,p([1 3]),p([2 4]),'-','Color',col,'LineWidth',.85);
 end
end
if g.class==1 && any(strcmp(layer,{'all','envelope'}))
 p=g.sweptEnvelope;patch(ax,p(:,1),p(:,2),[.25 .65 .88],'FaceAlpha',.17,'EdgeColor',[.13 .46 .70],'LineWidth',1.5);
 q=g.witness.best.reverse;plot(ax,q(:,1),q(:,2),'Color',[.04 .51 .32],'LineWidth',2.4);
 for k=unique(round(linspace(1,size(q,1),9))),cw.drawTruck(ax,q(k,:),scene.truck,[.13 .46 .7],.06);end
end
if ~isempty(g.anchors) && any(strcmp(layer,{'all','anchors'}))
 q=cw.segment(scene.goal,g.freeStem,0,.2);plot(ax,q(:,1),q(:,2),'--','Color',[.10 .49 .41],'LineWidth',1.4);
 a=g.anchors;scatter(ax,a(:,1),a(:,2),32,[.78 .30 .22],'filled');j=a(:,4)>0;scatter(ax,a(j,1),a(j,2),38,[.07 .60 .39],'filled');
 if ~isempty(g.visibleEnvelope),p=g.visibleEnvelope;patch(ax,p(:,1),p(:,2),[.29 .68 .52],'FaceAlpha',.10,'EdgeColor',[.10 .5 .37],'LineWidth',1.1);end
end
if ~isempty(g.turnProbes) && any(strcmp(layer,{'all','turns'}))
 for p=g.turnProbes
  q=p.q;col=[.83 .40 .16];if p.free,col=[.10 .45 .83];end
  plot(ax,q(:,1),q(:,2),'Color',col,'LineWidth',1.7);if ~p.free && ~isempty(q),plot(ax,q(end,1),q(end,2),'x','Color',col,'LineWidth',2,'MarkerSize',8);end
 end
end
if any(strcmp(layer,{'all','turns'})) && ~isempty(g.turnProbes)
 points=zeros(0,2);for probe=g.turnProbes,points=[points;probe.q(:,1:2)];end
 if ~isempty(points),lo=min(points)-scene.truck.front-1;hi=max(points)+scene.truck.front+1;xx=xlim(ax);yy=ylim(ax);xlim(ax,[min(xx(1),lo(1)) max(xx(2),hi(1))]);ylim(ax,[min(yy(1),lo(2)) max(yy(2),hi(2))]);end
end
cw.drawTruck(ax,scene.start,scene.truck,[.12 .38 .72],.85);cw.drawTruck(ax,scene.goal,scene.truck,[.11 .58 .38],.85);
end
