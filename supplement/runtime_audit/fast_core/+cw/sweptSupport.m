function support=sweptSupport(path,t,normals)
% Exact support of all four body corners along piecewise straight/circular arcs.
% Circle corner: C + Rot(theta)*[corner_x; corner_y-1/k].
% n'corner = n'C + A*cos(theta)+B*sin(theta); extrema are endpoints or atan2(B,A)+2*pi*j.
corners=[t.front t.width/2;t.front -t.width/2;-t.rear -t.width/2;-t.rear t.width/2];
support=-inf(size(normals,1),1);pose=path.start;
for j=1:numel(path.lengths)
 L=path.lengths(j);k=path.curvatures(j);a=pose(3);z=cw.segment(pose,L,k,max(L,1));last=z(end,:);
 if abs(k)<1e-12
  support=max(support,max(normals*[cw.vehicle(pose,t);cw.vehicle(last,t)]',[],2));
 else
  center=pose(1:2)+[-sin(a) cos(a)]/k;low=min(a,last(3));high=max(a,last(3));
  for c=1:4
   v=corners(c,:)-[0 1/k];A=normals(:,1)*v(1)+normals(:,2)*v(2);B=-normals(:,1)*v(2)+normals(:,2)*v(1);
   m=max(A*cos(low)+B*sin(low),A*cos(high)+B*sin(high));theta=atan2(B,A);theta=theta+2*pi*ceil((low-theta)/(2*pi));hit=theta<=high+1e-12;m(hit)=hypot(A(hit),B(hit));
   support=max(support,normals*center'+m);
  end
 end
 pose=last;
end
end
