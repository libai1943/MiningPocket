function draw(ax,scene,result,mode)
cla(ax,'reset');pieces=cb.pieces(result);[q,~,cusps]=cwr.sample(pieces,.3);lo=min([q(:,1:2);scene.goal(1:2);scene.start(1:2)])-[12 13];hi=max(q(:,1:2))+[12 13];
near=scene.boundary(scene.boundary(:,2)>=lo(2)&scene.boundary(:,2)<=hi(2),:);
if ~isempty(near),lo(1)=min(lo(1),min(near(:,1))-3);hi(1)=max(hi(1),max(near(:,1))+3);end
cw.drawScene(ax,scene,[lo(1) hi(1) lo(2) hi(2)]);
set(findobj(ax,'Type','text','String','停靠点'),'Clipping','on');set(findobj(ax,'Type','text','String','起点'),'Clipping','on');
if mode==1,return;end
if mode==3
 [count,bounds]=cb.drawCandidates(ax,result);setappdata(ax,'candidateCount',count);
 if all(isfinite(bounds(:))),xx=xlim(ax);yy=ylim(ax);xlim(ax,[min(xx(1),bounds(1,1)-3) max(xx(2),bounds(2,1)+3)]);ylim(ax,[min(yy(1),bounds(1,2)-3) max(yy(2),bounds(2,2)+3)]);end
end

for p=pieces
 qq=cwr.sample(p,.15);col=[.07 .52 .34];if p.gear==1,col=[.12 .39 .8];end
 if mode==3,plot(ax,qq(:,1),qq(:,2),'Color','w','LineWidth',5.5);end
 plot(ax,qq(:,1),qq(:,2),'Color',col,'LineWidth',2.4);
 if mode==2,for i=unique(round(linspace(1,size(qq,1),max(2,ceil(sum(p.lengths)/20))))),cw.drawTruck(ax,qq(i,:),scene.truck,col,.05);end;end
end
for i=1:size(cusps,1),p=cusps(i,:);plot(ax,p(1),p(2),'p','MarkerSize',11,'MarkerFaceColor',[1 .64 .1],'MarkerEdgeColor',[.7 .3 .05]);end
end
