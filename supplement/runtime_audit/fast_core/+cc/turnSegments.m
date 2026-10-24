function seg=turnSegments(phi,R,ramp)
% Segment coefficients need no displacement integration.
sigma=1/(R*ramp);peak=min(1/R,sqrt(abs(phi)*sigma));lr=peak/sigma;
lc=max(0,(abs(phi)-peak*lr)/max(peak,eps));k=sign(phi)*peak;u=sign(phi)*sigma;
seg=[lr 0 u;lc k 0;lr k -u];seg=seg(seg(:,1)>1e-9,:);
end
