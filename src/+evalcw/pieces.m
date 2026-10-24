function a=pieces(sol)
% [length, initial physical steering curvature, final curvature, gear].
a=[];
for j=1:numel(sol.legs)
 leg=sol.legs{j};v=leg.path.segments;z=[v(:,1),v(:,2),v(:,2)+v(:,3).*v(:,1),repmat(leg.gear,size(v,1),1)];
 if isfield(leg.path,'bodyOffset'),z(:,2:3)=leg.gear*z(:,2:3);end
 if leg.reverseOrder,z=flipud(z);z(:,2:3)=z(:,[3 2]);end
 a=[a;z(z(:,1)>1e-10,:)]; %#ok<AGROW>
end
end
