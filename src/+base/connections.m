function [p,costs]=connections(start,goal,R,o,kind)
% MATLAB Navigation Toolbox exact Reeds-Shepp words or both Dubins gears.
p={};costs=[];
if strcmp(kind,'dubins')
 for gear=[1 -1]
  a=start;b=goal;a(3)=a(3)+(gear<0)*pi;b(3)=b(3)+(gear<0)*pi;
  dc=dubinsConnection('MinTurningRadius',R);[paths,~]=connect(dc,a,b,'PathSegments','all');
  for j=1:numel(paths)
   z=paths{j};if ~isfinite(z.Length),continue;end
   k=zeros(size(z.MotionLengths));k(strcmp(z.MotionTypes,'L'))=gear/R;k(strcmp(z.MotionTypes,'R'))=-gear/R;
   v=[z.MotionLengths(:),k(:),k(:),gear*ones(numel(k),1)];v=v(v(:,1)>1e-8,:);p{end+1}=v;costs(end+1)=base.cost(v,o); %#ok<AGROW>
  end
 end
else
 rc=reedsSheppConnection('MinTurningRadius',R,'ForwardCost',1,'ReverseCost',o.reverseWeight);[paths,~]=connect(rc,start,goal,'PathSegments','all');
 for j=1:numel(paths)
  z=paths{j};if ~isfinite(z.Length),continue;end
  k=zeros(size(z.MotionLengths));k(strcmp(z.MotionTypes,'L'))=1/R;k(strcmp(z.MotionTypes,'R'))=-1/R;
  v=[z.MotionLengths(:),k(:),k(:),z.MotionDirections(:)];v=v(v(:,1)>1e-8,:);p{end+1}=v;costs(end+1)=base.cost(v,o); %#ok<AGROW>
 end
end
[costs,ix]=sort(costs);p=p(ix);
end
