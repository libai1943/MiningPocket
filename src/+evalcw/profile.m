function r=profile(p,truck,o)
% Exact constant-acceleration timing on an arc-length mesh. Every real
% steering jump is stopped; continuous steering obeys a shared rate bound.
if nargin<3,o=evalcw.options();end
assert(~isempty(p)&&all(p(:,1)>0));wb=truck.wheelbase;
blocks={};events=struct('distance',{},'shift',{},'jump',{},'deltaBefore',{},'deltaAfter',{},'steeringTime',{},'gearTime',{},'settleTime',{},'duration',{});
bs=0;bk=p(1,2);bc=Inf;bd=[];bg=p(1,4);globalS=0;
d0=atan(wb*p(1,2));if abs(d0-o.initialSteer)>o.kappaTolerance,events(end+1)=event(0,false,o.initialSteer,d0,o);end
for j=1:size(p,1)
 if j>1
  shift=p(j,4)~=p(j-1,4);jump=abs(p(j,2)-p(j-1,3))>o.kappaTolerance;
  if shift||jump
   blocks{end+1}=pack(bs,bk,bc,bd,bg); %#ok<AGROW>
   events(end+1)=event(globalS,shift,atan(wb*p(j-1,3)),atan(wb*p(j,2)),o); %#ok<AGROW>
   bs=0;bk=p(j,2);bc=Inf;bd=[];bg=p(j,4);
  end
 end
 n=max(2,ceil(p(j,1)/o.step));ds=p(j,1)/n;kk=linspace(p(j,2),p(j,3),n+1)';kp=(p(j,3)-p(j,2))/p(j,1);
 kMax=max(abs(kk(1:end-1)),abs(kk(2:end)));kMin=min(abs(kk(1:end-1)),abs(kk(2:end)));kMin(kk(1:end-1).*kk(2:end)<=0)=0;
 dPrime=abs(wb*kp)./(1+(wb*kMin).^2);
 vmax=o.vForward;if bg<0,vmax=o.vReverse;end
 cap=min(vmax,min(sqrt(o.aLateral./max(kMax,1e-30)),o.steerRate./max(dPrime,1e-30)));
 nodeCap=[cap(1);min(cap(1:end-1),cap(2:end));cap(end)];bc(end)=min(bc(end),nodeCap(1));
 bs=[bs;bs(end)+(1:n)'*ds];bk=[bk;kk(2:end)];bc=[bc;nodeCap(2:end)];bd=[bd;dPrime]; %#ok<AGROW>
 globalS=globalS+p(j,1);
end
blocks{end+1}=pack(bs,bk,bc,bd,bg);
dEnd=atan(wb*p(end,3));if abs(dEnd-o.finalSteer)>o.kappaTolerance,events(end+1)=event(globalS,false,dEnd,o.finalSteer,o);end
moving=0;allS=[];allV=[];allK=[];allT=[];offset=0;maxLat=0;maxRate=0;maxAcc=0;maxBrake=0;
for j=1:numel(blocks)
 b=blocks{j};v=b.cap;v(1)=0;v(end)=0;ds=diff(b.s);
 for k=1:numel(ds),v(k+1)=min(v(k+1),sqrt(v(k)^2+2*o.aAccelerate*ds(k)));end
 for k=numel(ds):-1:1,v(k)=min(v(k),sqrt(v(k+1)^2+2*o.aBrake*ds(k)));end
 dt=2*ds./(v(1:end-1)+v(2:end));assert(all(isfinite(dt)));b.v=v;b.t=[0;cumsum(dt)];blocks{j}=b;
 aa=diff(v.^2)./(2*ds);maxAcc=max(maxAcc,max(aa));maxBrake=max(maxBrake,max(-aa));
 maxLat=max(maxLat,max(max(v(1:end-1),v(2:end)).^2.*max(abs(b.k(1:end-1)),abs(b.k(2:end)))));
 maxRate=max(maxRate,max(max(v(1:end-1),v(2:end)).*b.dPrime));
 allS=[allS;b.s+offset];allV=[allV;v*b.gear];allK=[allK;b.k];allT=[allT;b.t+moving]; %#ok<AGROW>
 moving=moving+b.t(end);offset=offset+b.s(end);
end
if isempty(events),dwell=0;steer=0;shifts=0;jumps=0;else,dwell=sum([events.duration]);steer=sum([events.steeringTime]);shifts=sum([events.shift]);jumps=sum([events.jump]&~[events.shift]&[events.distance]>1e-8&[events.distance]<globalS-1e-8);end
r=struct('length',sum(p(:,1)),'forwardLength',sum(p(p(:,4)>0,1)),'reverseLength',sum(p(p(:,4)<0,1)),...
 'movingTime',moving,'dwellTime',dwell,'totalTime',moving+dwell,'stationarySteeringTime',steer,'gearChanges',shifts,'sameGearSteeringStops',jumps,...
 'internalStops',numel(blocks)-1,'maxAbsCurvature',max(max(abs(p(:,2:3)))),'maxLateralAcceleration',maxLat,'maxSteerRate',maxRate,'maxAcceleration',maxAcc,'maxBraking',maxBrake,...
 's',allS,'v',allV,'kappa',allK,'movingClock',allT,'blocks',{blocks},'events',events);
r.valid=maxLat<=o.aLateral+1e-8 && maxRate<=o.steerRate+1e-8 && maxAcc<=o.aAccelerate+1e-8 && maxBrake<=o.aBrake+1e-8;
end
function b=pack(s,k,cap,dPrime,gear)
b=struct('s',s,'k',k,'cap',cap,'dPrime',dPrime,'gear',gear);
end
function e=event(s,shift,before,after,o)
st=abs(after-before)/o.steerRate;gt=shift*o.shiftTime;settle=o.settleTime;
if strcmp(o.eventMode,'parallel'),duration=max(st,gt)+settle;else,duration=st+gt+settle;end
e=struct('distance',s,'shift',logical(shift),'jump',abs(after-before)>o.kappaTolerance,'deltaBefore',before,'deltaAfter',after,'steeringTime',st,'gearTime',gt,'settleTime',settle,'duration',duration);
end

