function seg=turnSegments(phi,R,ramp)
[~,~,p]=oldcc.turn(phi,R,ramp);seg=[p(1) 0 p(4);p(2) p(3) 0;p(1) p(3) -p(4)];
seg=seg(seg(:,1)>1e-9,:);
end
