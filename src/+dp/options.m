function o=options()
o.layerSpacing=22;o.coarseOffsets=-24:6:24;o.coarseHeadings=deg2rad(-30:15:30);
o.fineOffsets=-3:1.5:3;o.fineHeadingOffsets=deg2rad(-15:7.5:15);
o.maxEvaluations=1800;o.fineEvaluations=700;o.checkStep=.04;
o.radii=[1 1.3 1.7];o.ramps=[4 8];o.refine=true;
end
