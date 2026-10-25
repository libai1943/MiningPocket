function pose=endpoint(path)
% Endpoint only, with exactly the same segment recursion as cc.sample.
pose=path.start;
for j=1:size(path.segments,1)
 v=path.segments(j,:);L=v(1);z=cc.integrate(pose(3),v(2),v(3),L);
 pose=[pose(1)+real(z),pose(2)+imag(z),pose(3)+v(2)*L+.5*v(3)*L^2];
end
end
