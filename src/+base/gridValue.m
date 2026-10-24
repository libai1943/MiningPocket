function h=gridValue(m,q,target)
xy=round((q(:,1:2)-m.origin)/m.resolution)+1;xy=max([1 1],min(m.size,xy));ix=sub2ind(m.size,xy(:,1),xy(:,2));h=m.distance(ix);bad=~isfinite(h);h(bad)=vecnorm(q(bad,1:2)-target(1:2),2,2);
end
