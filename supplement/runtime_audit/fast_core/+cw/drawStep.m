function caption=drawStep(ax,scene,r,step)
cla(ax,'reset');s=r.best;xy=[scene.start(1:2);scene.goal(1:2)];
if r.success,xy=[xy;s.forward(:,1:2);s.reverse(:,1:2)];end
if step==3 || step==4
 for j=1:numel(r.curves),xy=[xy;r.curves{j}.q(:,1:2)];end %#ok<AGROW>
end
lo=min(xy)-[14 12];hi=max(xy)+[16 12];lim=[lo(1) hi(1) lo(2) hi(2)];
near=scene.boundary(scene.boundary(:,2)>=lo(2)&scene.boundary(:,2)<=hi(2),:);
if ~isempty(near),lim(1)=min(lim(1),min(near(:,1))-4);lim(2)=max(lim(2),max(near(:,1))+4);end
cw.drawScene(ax,scene,lim);
blue=[.10 .35 .75];green=[.02 .58 .37];orange=[.9 .48 .06];
switch step
 case 1
  caption=sprintf('%s：车头差角 %.1f°。灰虚线是已知地图范围，车身按实际尺寸绘制。',r.gate.name,abs(rad2deg(r.gate.headingDifference)));
 case 2
  if isfield(r,'terminalStraight'),q=r.terminalStraight;plot(ax,q(:,1),q(:,2),'Color',green,'LineWidth',3);cw.drawTruck(ax,q(end,:),scene.truck,green,.12);end
  caption=sprintf('先固定末端 %.2f 米直倒。计算时从停靠点沿车头方向向外生长。',scene.minimumStraight);
 case 3
  if isfield(r,'primaryTrace')&&isfield(r.primaryTrace,'startCurves'),for jj=1:numel(r.primaryTrace.startCurves),p=r.primaryTrace.startCurves{jj}.q;plot(ax,p(:,1),p(:,2),'Color',blue);end;end
  if isfield(r,'startCurves'),for jj=1:numel(r.startCurves),p=r.startCurves{jj}.q;plot(ax,p(:,1),p(:,2),'Color',blue);end;end
  if isfield(r,'primaryTails'),for jj=1:numel(r.primaryTails),p=cw.sample(r.primaryTails{jj},.5);plot(ax,p(:,1),p(:,2),'Color',[.65 .78 .72]);end;end
  if isfield(r,'bridgeTrace'),paths=[r.bridgeTrace.goalPaths r.bridgeTrace.startPaths];for jj=1:numel(paths),p=paths{jj};plot(ax,p(:,1),p(:,2),'Color',[.48 .68 .72]);end;end
  for j=1:numel(r.curves)
   c=r.curves{j};q=c.q;col=blue;if c.side>0,col=[.45 .64 .85];end
   plot(ax,q(:,1),q(:,2),'Color',col,'LineWidth',.85);
   if c.hit,plot(ax,q(end,1),q(end,2),'x','Color',[.85 .22 .18],'MarkerSize',5,'LineWidth',1.2);end
  end
  if isfield(r,'regionalTail'),for j=1:numel(r.regionalTail),q=r.regionalTail{j};plot(ax,q(:,1),q(:,2),'Color',[.63 .76 .88]);end;end
  caption='显示本轮记录的生长曲线与连接族；红叉是整车安全包络碰边时的后轴位置。';
 case 4
  q=r.candidates;if isfield(r,'primaryTrace'),q=[q;r.primaryTrace.candidates];end;if isfield(r,'bridgeTrace'),q=[q;[r.bridgeTrace.goalAnchors zeros(size(r.bridgeTrace.goalAnchors,1),2)];[r.bridgeTrace.startAnchors zeros(size(r.bridgeTrace.startAnchors,1),2)]];end
  if ~isempty(q)
   scatter(ax,q(:,1),q(:,2),12,orange,'filled');n=size(q,1);ix=1:n;
   quiver(ax,q(ix,1),q(ix,2),2.5*cos(q(ix,3)),2.5*sin(q(ix,3)),0,'Color',[.57 .30 .08],'LineWidth',.7,'MaxHeadSize',.5);
  end
  if isfield(r,'regionalPoints'),p=r.regionalPoints;scatter(ax,p(:,1),p(:,2),5,[.58 .65 .72]);end
  caption=sprintf('从最后安全位置沿弧长退回 %.1f 米，再按角度取点。橙点是真实候选换向点，小箭头表示车头方向。',r.options.backoff);
  if isfield(r,'regionalPoints'),caption='灰点是通道预备网格；橙点是倒车半段可达的候选，小箭头表示车头朝向。';end
 case 5
  for j=1:numel(r.feasible)
   v=r.feasible{j};q=v.forward;plot(ax,q(:,1),q(:,2),'Color',[.61 .74 .91],'LineWidth',.9);q=v.reverse;plot(ax,q(:,1),q(:,2),'Color',[.59 .80 .70],'LineWidth',.9);
  end
  caption='先检查倒车半段，再连接起点；仅显示通过碰撞检查、曾改善评分的完整候选。';
 case 6
  if r.success
   q=s.forward;plot(ax,q(:,1),q(:,2),'Color',blue,'LineWidth',3);q=s.reverse;plot(ax,q(:,1),q(:,2),'Color',green,'LineWidth',3);
   if ~isempty(s.cusp)
    plot(ax,s.cusp(1),s.cusp(2),'p','MarkerSize',15,'MarkerFaceColor',[1 .65 .1],'MarkerEdgeColor',[.6 .3 0]);
    cw.drawTruck(ax,s.cusp,scene.truck,orange,.12);text(ax,s.cusp(1)+4,s.cusp(2)+4,'唯一换向点','Color',[.7 .35 .03],'FontWeight','bold');
   end
   q=[s.forward;s.reverse];for j=unique(round(linspace(1,size(q,1),10))),cw.drawTruck(ax,q(j,:),scene.truck,[.35 .5 .6],.02);end
   caption=sprintf('蓝色前进 %.1f 米，绿色倒车 %.1f 米；换向 %d 次。末端保持直倒。',s.forwardLength,s.reverseLength,s.cuspCount);
  else
   caption='本轮有限候选集中未找到合格连接；该任务保留在基准中。';
  end
end
ax.FontSize=12;
end
