function z=integrate(theta,k,sigma,L)
% 16-point Gauss-Legendre quadrature for clothoid displacement.
% Vector inputs accepted. No trajectory optimization.
persistent x w
if isempty(x)
 n=16;b=(1:n-1)./sqrt(4*(1:n-1).^2-1);[V,D]=eig(diag(b,1)+diag(b,-1));
 [x,ix]=sort(diag(D));w=2*V(1,ix).^2;x=(x'+1)/2;w=w/2;
end
L=L(:);theta=theta(:);k=k(:);sigma=sigma(:);
% Exact elementary integrals for straight/circular pieces. The clothoid
% quadrature and every search parameter are unchanged.
if isscalar(sigma) && sigma==0 && isscalar(k) && isscalar(theta)
 if k==0,z=L.*exp(1i*theta);
 else,z=exp(1i*theta).*2.*sin(.5*k*L).*exp(.5i*k*L)./k;end
 return
end
s=L.*x;ang=theta+k.*s+.5*sigma.*s.^2;
z=L.*sum(exp(1i*ang).*w,2);
end
