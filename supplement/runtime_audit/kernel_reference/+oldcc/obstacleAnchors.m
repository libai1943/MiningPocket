function info=obstacleAnchors(scene,a,b)
% Cross sections normal to the start-goal chord. Empty intervals inform anchors.
% This is geometric guidance only, not a whole-body feasibility certificate.
o=cw.defaults();w=cw.world(scene,o);v=b(1:2)-a(1:2);D=norm(v);e=v/D;n=[-e(2) e(1)];
pp=w.points-a(1:2);u=pp*e';lat=pp*n';buffer=scene.truck.width/2+o.margin;
info=struct('stations',[],'intervals',{{}},'anchors',zeros(0,3),'axis',e,'normal',n,'origin',a(1:2),'blockedStations',[]);
for f=[.3 .45 .6 .75]
 z=f*D;vals=sort(lat(abs(u-z)<4));range=[-25 25];
 occupied=[vals-buffer vals+buffer];merged=zeros(0,2);
 for j=1:size(occupied,1)
  if isempty(merged)||occupied(j,1)>merged(end,2),merged(end+1,:)=occupied(j,:);else,merged(end,2)=max(merged(end,2),occupied(j,2));end
 end
 edges=[range(1) range(1)];free=zeros(0,2);cursor=range(1);
 for j=1:size(merged,1)
  if merged(j,2)<range(1)||merged(j,1)>range(2),continue;end
  if merged(j,1)>cursor,free(end+1,:)=[cursor min(range(2),merged(j,1))];end
  cursor=max(cursor,merged(j,2));
 end
 if cursor<range(2),free(end+1,:)=[cursor range(2)];end
 info.stations(end+1)=z;info.intervals{end+1}=free;
 if any(abs(vals)<buffer),info.blockedStations(end+1)=z;end
 for j=1:size(free,1)
  if diff(free(j,:))<1,continue;end
  offsets=unique([mean(free(j,:)),max(free(j,1)+.5,min(free(j,2)-.5,0))]);
  for off=offsets
   xy=a(1:2)+z*e+off*n;pose=[xy atan2(e(2),e(1))];
   if ~cw.collides(pose,w,.1),info.anchors(end+1,:)=pose;end
  end
 end
end
end
