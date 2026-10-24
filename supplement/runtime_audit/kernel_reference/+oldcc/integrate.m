function z=integrate(theta,k,sigma,L)
% 16-point Gauss-Legendre quadrature for clothoid displacement.
% Vector inputs accepted. No trajectory optimization.
persistent x w
if isempty(x)
 n=16;b=(1:n-1)./sqrt(4*(1:n-1).^2-1);[V,D]=eig(diag(b,1)+diag(b,-1));
 [x,ix]=sort(diag(D));w=2*V(1,ix).^2;x=(x'+1)/2;w=w/2;
end
L=L(:);theta=theta(:);k=k(:);sigma=sigma(:);
s=L.*x;ang=theta+k.*s+.5*sigma.*s.^2;
z=L.*sum(exp(1i*ang).*w,2);
end
