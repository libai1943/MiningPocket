function o=options()
o.maxSeconds=300;o.maxNodes=150000;o.step=.04;o.primitive=3;o.steers=[-1 -.5 0 .5 1];o.xyResolution=1;o.yawResolution=deg2rad(5);o.reverseWeight=1.35;o.switchCost=5;o.steerCost=2;
o.shotInterval=10;o.goalCandidates=100;o.biInflation=1.5;o.biDelta=1;o.biMu=20;o.biGamma=30;o.biNeighbours=8;
end
