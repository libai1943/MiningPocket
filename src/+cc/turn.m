function [z,total,params]=turn(phi,R,ramp)
% A zero-curvature to zero-curvature turn: clothoid / optional circle / clothoid.
% Sign from phi; ramp is ramp length at peak curvature 1/R.
a=abs(phi(:));sg=sign(phi(:));sigma=1/(R*ramp);
peak=min(1/R,sqrt(a*sigma));lr=peak/sigma;
lc=max(0,(a-peak.*lr)./max(peak,eps));
k=sg.*peak;u=sg*sigma;
z1=cc.integrate(0,0,u,lr);th1=.5*u.*lr.^2;
z2=zeros(size(a));nz=abs(k)>1e-14;
z2(nz)=exp(1i*th1(nz)).*(exp(1i*k(nz).*lc(nz))-1)./(1i*k(nz));
th2=th1+k.*lc;z3=cc.integrate(th2,k,-u,lr);
z=z1+z2+z3;total=2*lr+lc;
params=[lr lc k u];
end
