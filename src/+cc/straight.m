function p=straight(q,L)
p=struct('start',q,'goal',q+[L*cos(q(3)),L*sin(q(3)),0],'segments',[L 0 0],'length',L);
end
