function [q,s,k]=execution(leg,step)
[q,s,k]=cc.sample(leg.path,step);
if isfield(leg.path,'bodyOffset')
 q(:,3)=q(:,3)-leg.path.bodyOffset;k=leg.gear*k;
end
if leg.reverseOrder,q=flipud(q);s=leg.path.length-flipud(s);k=flipud(k);end
end
