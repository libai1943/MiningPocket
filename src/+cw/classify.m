function [category,name,g]=classify(scene,o)
% Public classification uses only current input geometry and a reverse certificate.
if nargin<2,o=cw.defaults();end
g=cw.gate(scene,o);category=g.class;name=g.name;
end
