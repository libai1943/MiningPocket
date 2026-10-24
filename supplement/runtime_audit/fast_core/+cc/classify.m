function g=classify(scene)
% INPUT ONLY. Label never depends on the success of a planning routine.
a=abs(atan2(sin(scene.start(3)-scene.goal(3)),cos(scene.start(3)-scene.goal(3))));
% Roundoff only: put exact 45 and 135 degree boundaries on the lower side.
if abs(a-pi/4)<1e-12,a=pi/4;end
if abs(a-3*pi/4)<1e-12,a=3*pi/4;end
membership=[a<=pi/4,a>pi/4 && a<=3*pi/4,a>3*pi/4];
names={'同向到达','侧向到达','对向到达'};c=find(membership);
g=struct('class',c,'name',names{c},'angle',a,'membership',membership);
assert(sum(membership)==1);
end
