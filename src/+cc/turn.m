function [z,total,params]=turn(phi,R,ramp)
% Symmetric clothoid ramps with vanishing sharpness at vanishing angle.
persistent limits
if isempty(limits),limits=cc.numerics();end
a=abs(phi(:));a(a<=limits.zeroAngle)=0;sg=sign(phi(:));
sigma=(1/(R*ramp))*min(1,sqrt(a/limits.angleRegularization));
peak=min(1/R,sqrt(a.*sigma));lr=peak./max(sigma,realmin);
lc=max(0,(a-peak.*lr)./max(peak,realmin));k=sg.*peak;u=sg.*sigma;
z1=cc.integrate(0,0,u,lr);th1=.5*u.*lr.^2;
v=.5*k.*lc;z2=exp(1i*(th1+v)).*(lc.*sinc_local(v));
z=z1+z2+exp(1i*(sg.*a)).*conj(z1);
if nargout>1,total=2*lr+lc;end
if nargout>2,params=[lr lc k u];end
end
function v=sinc_local(x)
v=ones(size(x));nz=x~=0;v(nz)=sin(x(nz))./x(nz);
end
