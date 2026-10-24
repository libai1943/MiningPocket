function o=options()
% Finite representation; no obstacle-specific branches, no A*, no optimizer.
o.radiusFactors=[1 1.3 1.7];o.rampLengths=[4 8];
o.stations=[.30 .45 .60 .75];o.offsetFactors=[0 -.4 .4 -.8 .8 -1.2 1.2];
o.headingOffsets=deg2rad([0 -15 15]);o.maxTurn=.98*pi;o.checkStep=.04;
end
