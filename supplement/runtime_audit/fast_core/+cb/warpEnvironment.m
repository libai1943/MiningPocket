function q=warpEnvironment(q,p)
% Continuous inlet wall deformation. Keep the last 24 m of the goal lane fixed.
y=q(:,2);b=p.side*p.bendAmplitude*(exp(-((y-p.bendCenter)/p.bendWidth).^2)-exp(-(p.bendCenter/p.bendWidth)^2));weight=1-exp(-(max(y-24,0)/22).^2);q(:,1)=(1+(p.xScale-1)*weight).*q(:,1)+b.*weight;q(:,2)=y+(p.yScale-1)*max(y-24,0);
end
