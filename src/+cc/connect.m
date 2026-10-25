function paths=connect(a,b,radii,ramps,maxTurn)
% Two G2 turns and their common tangent. Solve ONE scalar tangent equation.
% No optimizer. Finite heading scan brackets roots; not a completeness claim.
if nargin<5,maxTurn=2*pi-.02;end
persistent rootOptions
if isempty(rootOptions),rootOptions=optimset('TolX',1e-11,'Display','off');end
paths={};delta=complex(b(1)-a(1),b(2)-a(2));
for R=radii
 for ramp=ramps
  for side1=[-1 1]
   for side2=[-1 1]
    cuts=unique(sort([-pi pi atan2(sin(a(3)),cos(a(3))) atan2(sin(b(3)),cos(b(3)))]));
    roots=[];
    for c=1:numel(cuts)-1
     xx=linspace(cuts(c)+1e-8,cuts(c+1)-1e-8,max(3,ceil((cuts(c+1)-cuts(c))/(pi/72))+1));
     [ff,~,ph1,ph2]=residual(xx);
     valid=abs(ph1)<=maxTurn & abs(ph2)<=maxTurn;
     for jj=1:numel(xx)-1
      if ~valid(jj)||~valid(jj+1),continue;end
      if ff(jj)*ff(jj+1)<0
       r=fzero(@scalar,[xx(jj) xx(jj+1)],rootOptions);roots(end+1)=r; %#ok<AGROW>
      elseif abs(ff(jj))<1e-8,roots(end+1)=xx(jj);end %#ok<AGROW>
     end
    end
    % Include degenerate zero turns at endpoint headings and chord heading.
    special=[a(3) b(3) angle(delta)];
    for r=special
     [v,~,p1,p2]=residual(r);if abs(v)<1e-7 && abs(p1)<=maxTurn && abs(p2)<=maxTurn,roots(end+1)=r;end %#ok<AGROW>
    end
    exact=[];for v=special,[f0,~,p0,p1]=residual(v);if abs(f0)<1e-9 && abs(p0)<=maxTurn && abs(p1)<=maxTurn,exact(end+1)=v;end;end
    roots=cc.uniqueRoots([exact roots],1e-10);
    for th=roots(:)'
     [f,d,p1,p2]=residual(th);if ~isscalar(d),error('cc:connect:NonScalarTangent','Tangent length must be scalar');end;if d<-1e-7||abs(f)>1e-6||abs(p1)>maxTurn||abs(p2)>maxTurn,continue;end
     seg=[cc.turnSegments(p1,R,ramp);max(0,d) 0 0;cc.turnSegments(p2,R,ramp)];seg=seg(seg(:,1)>1e-8,:);
     path=struct('start',a,'goal',b,'segments',seg,'length',sum(seg(:,1)),'radius',R,'ramp',ramp,'middleHeading',th,'turnAngles',[p1 p2],'tangentLength',max(0,d));
     if isempty(seg),continue;end
     qq=cc.endpoint(path);err=norm(qq(end,1:2)-b(1:2));
     if err<1e-5 && abs(atan2(sin(qq(end,3)-b(3)),cos(qq(end,3)-b(3))))<1e-7,paths{end+1}=path;end %#ok<AGROW>
    end
   end
  end
 end
end
if ~isempty(paths),[~,ix]=sort(cellfun(@(p)p.length,paths));paths=paths(ix);end
 function value=scalar(thetaValue),value=residual(thetaValue);end
 function [res,along,phiA,phiB]=residual(thetaValues)
  thetaValues=thetaValues(:);phiA=side1*mod(side1*(thetaValues-a(3)),2*pi);phiB=side2*mod(side2*(b(3)-thetaValues),2*pi);
  phiA(abs(phiA-2*pi*side1)<1e-9)=0;phiB(abs(phiB-2*pi*side2)<1e-9)=0;
  dispA=cc.turn(phiA,R,ramp)*exp(1i*a(3));dispB=cc.turn(phiB,R,ramp).*exp(1i*thetaValues);
  localDisplacement=(delta-dispA-dispB).*exp(-1i*thetaValues);res=imag(localDisplacement);along=real(localDisplacement);
 end
end

