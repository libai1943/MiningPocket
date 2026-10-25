function [J,e]=score(sol,truck)
o=evalcw.options();e=evalcw.profile(evalcw.pieces(sol),truck,o);
J=e.totalTime+o.stopPenalty*e.internalStops;if ~e.valid,J=Inf;end
end
