function o=defaults()
% All lengths are metres; the pose reference is the rear axle.
o.margin=.5; o.step=.65; o.fineStep=.10; o.stemStep=4;
o.radiusFactors=[1 1.15 1.4 1.8 2.4]; o.maxTurn=deg2rad(155);
o.backoff=1.0; o.angleStep=deg2rad(6); o.maxStem=70;
o.reverseWeight=1.35; o.maxCandidates=3200; o.record=true;
o.gateBuffer=0; o.gateNormals=32; o.gateAngle=pi/4;
o.mode='adaptive'; o.startDistance=60;
end
