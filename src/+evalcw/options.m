function o=options()
% Shared simulation parameters; not calibrated truck measurements.
o.step=.1;o.vForward=4;o.vReverse=2;o.aAccelerate=.5;o.aBrake=.7;o.aLateral=1;
o.steerRate=deg2rad(5);o.shiftTime=2;o.settleTime=.5;o.eventMode='parallel';
o.kappaTolerance=1e-9;o.initialSteer=0;o.finalSteer=0;
end
