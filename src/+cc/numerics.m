function n=numerics()
% Geometry tolerances shared by every continuous-curvature planner.
n.zeroAngle=64*eps;n.angleRegularization=1e-6;n.rootMerge=1e-10;
n.positionTolerance=1e-5;n.headingTolerance=1e-7;
end
