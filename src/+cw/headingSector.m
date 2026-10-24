function [category,name]=headingSector(scene)
a=scene.start(3)-scene.goal(3);c=cos(a);d=sin(a);
if c>abs(d)+1e-12,category=1;name='同向对齐';elseif c<-abs(d)-1e-12,category=3;name='对向回转';else,category=2;name='侧向切入';end
end
