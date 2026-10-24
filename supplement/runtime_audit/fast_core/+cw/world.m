function w=world(scene,o)
p=scene.boundary;
% A parked truck is a solid obstacle, including its interior.
if ~isempty(scene.parkedTruck)
 t=scene.truck;[xx,yy]=meshgrid(-t.rear:.25:t.front,-t.width/2:.25:t.width/2);
 q=scene.parkedTruck;R=[cos(q(3)) -sin(q(3));sin(q(3)) cos(q(3))];
 p=[p;[xx(:),yy(:)]*R'+q(1:2)];
end
% Fixed spatial bins retain original obstacle samples without closing unrelated outlines.
w.points=p;w.cellSize=8;w.origin=min(p)-15;w.size=ceil((max(p)-w.origin+15)/w.cellSize)+1;
ix=floor((p-w.origin)/w.cellSize)+1;id=sub2ind(w.size,ix(:,1),ix(:,2));
w.bins=accumarray(id,(1:size(p,1))',[prod(w.size) 1],@(x){x},{[]});
h=scene.boundary(convhull(scene.boundary(:,1),scene.boundary(:,2)),:);e=diff(h);
w.normals=[e(:,2) -e(:,1)];w.normals=w.normals./vecnorm(w.normals,2,2);
w.limits=sum(w.normals.*h(1:end-1,:),2);w.hull=h;
w.truck=scene.truck;w.margin=o.margin;
end
