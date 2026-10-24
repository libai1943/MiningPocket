function drawScene(ax,scene,limits)
hold(ax,'on');p=scene.boundary;scatter(ax,p(:,1),p(:,2),3,[.30 .34 .38],'filled','HandleVisibility','off');
h=p(convhull(p(:,1),p(:,2)),:);plot(ax,h(:,1),h(:,2),'--','Color',[.58 .60 .64],'LineWidth',.8,'HandleVisibility','off');
if ~isempty(scene.parkedTruck)
 v=cw.vehicle(scene.parkedTruck,scene.truck);patch(ax,v(:,1),v(:,2),[.65 .67 .70],'FaceAlpha',.6,'EdgeColor',[.3 .3 .3],'HandleVisibility','off');
end
cw.drawTruck(ax,scene.start,scene.truck,[.12 .38 .72],1);cw.drawTruck(ax,scene.goal,scene.truck,[.11 .58 .38],1);
text(ax,scene.start(1)+4,scene.start(2),'起点','Color',[.10 .3 .6],'FontSize',11,'FontWeight','bold');
text(ax,scene.goal(1)+4,scene.goal(2)-2,'停靠点','Color',[.05 .4 .24],'FontSize',11,'FontWeight','bold');
axis(ax,'equal');if nargin>2,xlim(ax,limits(1:2));ylim(ax,limits(3:4));end
grid(ax,'on');ax.GridAlpha=.10;ax.FontName='Microsoft YaHei';ax.FontSize=10;ax.Box='off';
xlabel(ax,'横向 / 米');ylabel(ax,'纵向 / 米');
end
