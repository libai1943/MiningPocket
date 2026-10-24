function [count,bounds]=drawCandidates(ax,tr)
% Plot all retained enumeration poses and generating steering curves.
count=0;bounds=[inf inf;-inf -inf];
for field={'curves','startCurves'}
 if isfield(tr,field{1})
  for i=1:numel(tr.(field{1})),p=tr.(field{1}){i};include(p.q);plot(ax,p.q(:,1),p.q(:,2),'Color',[.58 .73 .73],'LineWidth',.45);end
 end
end
if isfield(tr,'regionalTail'),for p=tr.regionalTail,q=p{1};include(q);plot(ax,q(:,1),q(:,2),'Color',[.68 .76 .66],'LineWidth',.4);end;end
if isfield(tr,'regionalPoints')&&~isempty(tr.regionalPoints),q=tr.regionalPoints;include(q);scatter(ax,q(:,1),q(:,2),5,[.62 .67 .7],'filled');end
if isfield(tr,'candidates')&&~isempty(tr.candidates)
 q=tr.candidates;include(q);count=count+size(q,1);scatter(ax,q(:,1),q(:,2),8,[.08 .50 .37],'filled');quiver(ax,q(:,1),q(:,2),cos(q(:,3)),sin(q(:,3)),0,'Color',[.12 .52 .4],'LineWidth',.4);
end
if isfield(tr,'bridgeTrace')
 b=tr.bridgeTrace;
 for field={'goalPaths','startPaths'},if isfield(b,field{1}),for p=b.(field{1}),q=p{1};include(q);plot(ax,q(:,1),q(:,2),'Color',[.65 .70 .78],'LineWidth',.4);end;end;end
 for field={'goalAnchors','startAnchors'},if isfield(b,field{1})&&~isempty(b.(field{1})),q=b.(field{1});include(q);scatter(ax,q(:,1),q(:,2),9,[.68 .42 .13],'filled');count=count+size(q,1);end;end
end
if isfield(tr,'primaryTrace'),[n,b]=cb.drawCandidates(ax,tr.primaryTrace);count=count+n;include(b);end
if isfield(tr,'recursion')
 for ff=tr.recursion.fans,for p=ff.fan,include(p.q);plot(ax,p.q(:,1),p.q(:,2),'Color',[.72 .62 .43],'LineWidth',.6);end;end
 nodes=tr.recursion.nodes;q=vertcat(nodes(2:end).pose);include(q);scatter(ax,q(:,1),q(:,2),18,[.84 .43 .08],'filled');count=count+size(q,1);
 for i=2:numel(nodes),if nodes(i).tested&&~isempty(nodes(i).trace),[n,b]=cb.drawCandidates(ax,nodes(i).trace);count=count+n;include(b);end;end
end
 function include(q)
  if isempty(q),return;end;q=q(all(isfinite(q(:,1:2)),2),1:2);if isempty(q),return;end;bounds(1,:)=min(bounds(1,:),min(q));bounds(2,:)=max(bounds(2,:),max(q));
 end
end
