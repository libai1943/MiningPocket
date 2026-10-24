function p=vehicle(q,t,margin)
if nargin<3,margin=0;end
v=[t.front+margin t.width/2+margin;t.front+margin -t.width/2-margin;-t.rear-margin -t.width/2-margin;-t.rear-margin t.width/2+margin];
R=[cos(q(3)) -sin(q(3));sin(q(3)) cos(q(3))];p=v*R'+q(1:2);
end
