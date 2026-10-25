function o=options()
o.directRadiusFactors=[1 1.3 1.7];o.directRampLengths=[4 8];
o.branchPolicy='compare';o.enabledBranches=[2 3];o.fixedOrder=[];o.headingThreshold=deg2rad(135);o.refine=true;o.feasibleLimit=3;o.maxSeconds=900;
o.scaleLevels=3;o.childScaleLevels=1;o.fineAngles=deg2rad(0:150);o.fineRampLength=1;o.fineConnectorRadiusFactors=[1 1.4];o.fanRadiusFactors=[1 1.4];o.expandedRadiusFactors=[1 1.4 1.8 2.4];o.fanRampLengths=[4 8];o.fanAngles=deg2rad(0:15:150);
o.goalOffsets=[0 8 16];o.startOffsets=[0 8];o.connectorRampLengths=4;o.connectorMaxTurn=1.8*pi;
o.reverseWeight=1.35;o.step=.04;o.maxDepth=2;o.maxNodes=128;o.maxCusps=3;
o.prefixLengths=[4 8 12];o.prefixRamp=1;o.positionBin=1e-6;o.angleBin=1e-8;
end

