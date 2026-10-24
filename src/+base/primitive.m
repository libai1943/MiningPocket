function [q,p]=primitive(start,L,k,g,step)
if nargin<5,step=.04;end
s=linspace(0,L,max(2,ceil(L/step)+1))';a=start(3)+g*k*s;
if abs(k)<1e-12,q=[start(1)+g*s*cos(start(3)),start(2)+g*s*sin(start(3)),a];else,q=[start(1)+(sin(a)-sin(start(3)))/k,start(2)-(cos(a)-cos(start(3)))/k,a];end
p=[L,k,k,g];
end
