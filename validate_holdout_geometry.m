function report=validate_holdout_geometry()
% Exact polygon/circle obstacle geometry with between-sample body allowance.
root=miningpocket_setup();report=struct([]);
for i=1:12
 d=load(fullfile(root,'benchmark/holdout',sprintf('H%03d.mat',i)));sc=d.scene;
 for method={'BRS','HA','BIAGT','GHA','CC-A*'}
  m=method{1};path=fullfile(root,'results/holdout',[strrep(m,'*','star') '_' sc.id '.mat']);v=load(path,'r','row');
  sampled=v.row.success;if isfield(v.row,'sampledMapSuccess')&&~isempty(v.row.sampledMapSuccess),sampled=v.row.sampledMapSuccess;end
  if ~sampled,continue;end
  pass=true;count=0;
  t=sc.truck;p=sc.workspacePolygon;p=[p;p(1,:)];
  parked=vertices(sc.parkedTruck,t.front,t.rear,t.width/2);parked=[parked;parked(1,:)];
  for j=1:numel(v.r.best.legs)
   [q,s,k]=cm.execution(v.r.best.legs{j},.02);rho=hypot(t.front+.5,t.width/2+.5);
   delta=max(diff(s))/2*(1+rho*max(abs(k)))+1e-5;f=t.front+.5+delta;b=t.rear+.5+delta;hw=t.width/2+.5+delta;
   cs=cos(q(:,3));sn=sin(q(:,3));xy=[f hw;f -hw;-b -hw;-b hw];
   xx=q(:,1)+cs*xy(:,1)'-sn*xy(:,2)';yy=q(:,2)+sn*xy(:,1)'+cs*xy(:,2)';
   inside=inpolygon(xx,yy,p(:,1),p(:,2));pass=pass&&all(inside(:));
   pass=pass&&~any(hits(p,q,f,b,hw))&&~any(hits(parked,q,f,b,hw));
   for n=1:4
    dx=parked(n,1)-q(:,1);dy=parked(n,2)-q(:,2);lx=dx.*cs+dy.*sn;ly=-dx.*sn+dy.*cs;pass=pass&&~any(lx>=-b & lx<=f & abs(ly)<=hw);
   end
   inpk=inpolygon(xx,yy,parked(:,1),parked(:,2));pass=pass&&~any(inpk(:));
   ci=sc.exclusionCircle;dx=ci(1)-q(:,1);dy=ci(2)-q(:,2);lx=dx.*cs+dy.*sn;ly=-dx.*sn+dy.*cs;
   distance=hypot(lx-min(f,max(-b,lx)),ly-min(hw,max(-hw,ly)));pass=pass&&all(distance>ci(3));
   count=count+size(q,1);
  end
  report=[report struct('id',sc.id,'method',m,'passed',pass,'samples',count,'geometry','continuous polygon, circle, parked rectangle')];
 end
end
% Independent source-geometry validation is part of held-out task success.
rows=jsondecode(fileread(fullfile(root,'results/holdout/holdout12.json')));rows=rows(:)';
for k=1:numel(rows)
 row=rows(k);sampled=row.success;if isfield(row,'sampledMapSuccess')&&~isempty(row.sampledMapSuccess),sampled=row.sampledMapSuccess;end
 match=[];if ~isempty(report),match=find(strcmp({report.id},row.id)&strcmp({report.method},row.method),1);end
 passed=false;if ~isempty(match),passed=report(match).passed;end
 rows(k).sampledMapSuccess=sampled;rows(k).continuousGeometryPassed=passed;rows(k).success=sampled&&passed;
 target=fullfile(root,'results/holdout',[strrep(row.method,'*','star') '_' row.id '.mat']);record=load(target);record.row=rows(k);save(target,'-struct','record','-v7');
end
evalcw.writeJson(fullfile(root,'results/holdout/holdout12.json'),rows);
evalcw.writeJson(fullfile(root,'qa/holdout_continuous_geometry.json'),report);
certified=0;if ~isempty(report),certified=sum([report.passed]);end
fprintf('Continuous holdout geometry: %d/%d valid trajectories certified.\n',certified,numel(report));
end
function out=hits(p,q,f,b,hw)
cs=cos(q(:,3));sn=sin(q(:,3));out=false(size(q,1),1);
for j=1:size(p,1)-1
 dx1=p(j,1)-q(:,1);dy1=p(j,2)-q(:,2);dx2=p(j+1,1)-q(:,1);dy2=p(j+1,2)-q(:,2);
 a=[dx1.*cs+dy1.*sn,-dx1.*sn+dy1.*cs];d=[dx2.*cs+dy2.*sn,-dx2.*sn+dy2.*cs]-a;
 lo=zeros(size(q,1),1);hi=ones(size(lo));valid=true(size(lo));
 for axis=1:2
  mn=-b;mx=f;if axis==2,mn=-hw;mx=hw;end
  zero=abs(d(:,axis))<1e-14;valid=valid&~(zero&(a(:,axis)<mn|a(:,axis)>mx));
  denominator=d(:,axis);denominator(zero)=1;
  t1=(mn-a(:,axis))./denominator;t2=(mx-a(:,axis))./denominator;
  ll=min(t1,t2);hh=max(t1,t2);ll(zero)=-Inf;hh(zero)=Inf;
  lo=max(lo,ll);hi=min(hi,hh);
 end
 out=out|(valid&lo<=hi);
end
end
function p=vertices(q,f,b,h)
xy=[f h;f -h;-b -h;-b h];a=q(3);p=xy*[cos(a) sin(a);-sin(a) cos(a)]+q(1:2);
end

