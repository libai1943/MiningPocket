function [q,s,k]=sample(path,step)
% Rows of segments: [length initial curvature curvature derivative].
pose=path.start;q=pose;s=0;k=0;dist=0;
for j=1:size(path.segments,1)
 v=path.segments(j,:);L=v(1);ss=linspace(0,L,max(2,ceil(L/step)+1))';
 z=cc.integrate(pose(3),v(2),v(3),ss);
 qq=[pose(1)+real(z),pose(2)+imag(z),pose(3)+v(2)*ss+.5*v(3)*ss.^2];
 if j==1,k(1)=v(2);end
 q=[q;qq(2:end,:)];s=[s;dist+ss(2:end)];k=[k;v(2)+v(3)*ss(2:end)]; %#ok<AGROW>
 pose=qq(end,:);dist=dist+L;
end
end
