function view_case(caseNumber, method)
% Plot a freshly computed solution, or the fixed benchmark without a result.
if nargin < 2, method = 'BRS'; end
root = miningpocket_setup(); id = sprintf('B%03d',caseNumber);
d = load(fullfile(root,'benchmark','cases',[id '.mat']),'scene'); s = d.scene;
figure('Color','w'); ax = axes; hold(ax,'on');
plot(ax,s.boundary(:,1),s.boundary(:,2),'.','Color',[.58 .56 .53],'MarkerSize',3);
body(s.start,s.truck,[.2 .55 .78]); body(s.goal,s.truck,[.85 .43 .17]);
if ~isempty(s.parkedTruck), body(s.parkedTruck,s.truck,[.45 .45 .45]); end
f = fullfile(root,'results','reproduced',[upper(char(method)) '_' id '.mat']);
if isfile(f)
    a = load(f,'result');
    if a.result.success
        legs = a.result.planner.best.legs;
        for k = 1:numel(legs)
            q = cm.execution(legs{k},.1); c = [.05 .55 .43];
            if legs{k}.gear < 0, c = [.18 .35 .76]; end
            plot(ax,q(:,1),q(:,2),'Color',c,'LineWidth',1.8);
        end
    end
end
axis(ax,'equal'); grid(ax,'on'); xlabel('x (m)'); ylabel('y (m)');
title(sprintf('Case #%d',caseNumber)); set(ax,'FontName','Times New Roman');
end
function body(q,t,color)
v = [t.front t.width/2;t.front -t.width/2;-t.rear -t.width/2;-t.rear t.width/2];
rot = [cos(q(3)) -sin(q(3));sin(q(3)) cos(q(3))];
v = v*rot'+q(1:2); patch(v(:,1),v(:,2),color,'FaceAlpha',.16,'EdgeColor',color);
end
