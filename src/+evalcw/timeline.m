function z=timeline(e,truck)
% Wall clock includes every stationary steering/gear/settling event.
t=[];v=[];delta=[];s=[];clock=0;offset=0;used=false(1,numel(e.events));
for j=1:numel(e.blocks)
 for k=1:numel(e.events)
  if ~used(k)&&abs(e.events(k).distance-offset)<1e-6,addEvent(e.events(k));used(k)=true;end
 end
 b=e.blocks{j};t=[t;clock+b.t];v=[v;b.gear*b.v];delta=[delta;atan(truck.wheelbase*b.k)];s=[s;offset+b.s];clock=clock+b.t(end);offset=offset+b.s(end); %#ok<AGROW>
end
for k=1:numel(e.events),if ~used(k),addEvent(e.events(k));end;end
z=struct('t',t,'v',v,'delta',delta,'s',s);assert(abs(clock-e.totalTime)<1e-7);
 function addEvent(ev)
  dt=unique([0 ev.steeringTime ev.duration])';a=min(1,dt/max(ev.steeringTime,eps));t=[t;clock+dt];v=[v;zeros(size(dt))];delta=[delta;ev.deltaBefore+(ev.deltaAfter-ev.deltaBefore)*a];s=[s;repmat(ev.distance,size(dt))];clock=clock+ev.duration;
 end
end
