function [free,minClearance,lines]=cornerLinks(scene,w,margin)
a=cw.vehicle(scene.start,scene.truck);b=cw.vehicle(scene.goal,scene.truck);lines=zeros(16,4);free=false(4);minClearance=inf(4);
for i=1:4,for j=1:4
 d=b(j,:)-a(i,:);pp=w.points-a(i,:);u=max(0,min(1,(pp*d')/max(sum(d.^2),eps)));dd=pp-u.*d;
 minClearance(i,j)=sqrt(min(sum(dd.^2,2)));domain=min(w.limits'-[a(i,:);b(j,:)]*w.normals',[],'all');
 free(i,j)=minClearance(i,j)>=margin+1e-9 && domain>=margin+1e-9;lines(4*(i-1)+j,:)=[a(i,:) b(j,:)];
end,end
end
