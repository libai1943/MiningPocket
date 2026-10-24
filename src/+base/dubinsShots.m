function out=dubinsShots(start,goal,w,o)
% The two shortest Dubins connections: one construction gear each.
% Matches C-MINLP Algorithm 1's two connection proposals; collect both.
[pool,~]=base.connections(start,goal,w.truck.radius,o,'dubins');out={};
for gear=[1 -1]
 idx=find(cellfun(@(p)all(p(:,4)==gear),pool));if isempty(idx),continue;end
 [~,ii]=min(cellfun(@(p)sum(p(:,1)),pool(idx)));p=pool{idx(ii)};q=start;valid=true;
 for k=1:size(p,1)
  qq=base.primitive(q,p(k,1),p(k,2),p(k,4),o.step);if ~base.free(qq,w,p(k,2),p(k,1)/(size(qq,1)-1)),valid=false;break;end;q=qq(end,:);
 end
 if valid&&norm(q(1:2)-goal(1:2))<1e-5&&abs(atan2(sin(q(3)-goal(3)),cos(q(3)-goal(3))))<1e-7,out{end+1}=p;end %#ok<AGROW>
end
end
