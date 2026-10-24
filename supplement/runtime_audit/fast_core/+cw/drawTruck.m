function drawTruck(ax,q,t,col,alpha)
if nargin<5,alpha=.15;end
v=cw.vehicle(q,t);patch(ax,v(:,1),v(:,2),col,'FaceAlpha',alpha,'EdgeColor',col,'LineWidth',1.0,'HandleVisibility','off');
arrowColor=col;if alpha>.8,arrowColor=[1 1 1];end
quiver(ax,q(1),q(2),t.length*.55*cos(q(3)),t.length*.55*sin(q(3)),0,'Color',arrowColor,'LineWidth',1.5,'MaxHeadSize',.7,'HandleVisibility','off');
end
