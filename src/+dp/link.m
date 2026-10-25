function paths=link(a,b,R,Lc)
% Zero-curvature interfaces. Vectorized scalar closure, cached in local frame.
persistent memo
if isempty(memo),memo=containers.Map('KeyType','char','ValueType','any');end
d=b(1:2)-a(1:2);key=reshape(num2hex([d,a(3),b(3),R,Lc])',1,[]);
if isKey(memo,key),raw=memo(key);else
 raw={};ts=a(3);te=b(3);betas=unique([linspace(-pi/2+.02,pi/2-.02,41),ts,te,atan2(d(2),d(1))]);
 vals=res(betas);roots=[];
 for j=1:numel(betas)-1
  if abs(vals(j))<1e-9,roots(end+1)=betas(j);end
  if vals(j)*vals(j+1)<0,roots(end+1)=fzero(@res,[betas(j),betas(j+1)]);end
 end
 if abs(vals(end))<1e-9,roots(end+1)=betas(end);end
 special=[ts te atan2(d(2),d(1))];vv=res(special);roots=cc.uniqueRoots([special(abs(vv(:)')<1e-9) roots],1e-10);
 for beta=roots(:)'
  [f,along]=res(beta);if along < -1e-7 || abs(f)>1e-6,continue;end
  seg=[cc.turnSegments(beta-ts,R,Lc);max(0,along),0,0;cc.turnSegments(te-beta,R,Lc)];seg=seg(seg(:,1)>1e-9,:);
  if isempty(seg),continue;end
  p=struct('start',[0 0 ts],'goal',[d te],'segments',seg,'length',sum(seg(:,1)),'radius',R,'ramp',Lc,'middleHeading',beta,'turnAngles',[beta-ts te-beta],'tangentLength',max(0,along));
  raw{end+1}=p;
 end
 if ~isempty(raw),[~,ix]=sort(cellfun(@(p)p.length,raw));raw=raw(ix);end
 memo(key)=raw;
end
paths=raw;for j=1:numel(paths),paths{j}.start=a;paths{j}.goal=b;end
 function [f,along]=res(beta)
  z=cc.turn(beta(:)-ts,R,Lc)*exp(1i*ts)+cc.turn(te-beta(:),R,Lc).*exp(1i*beta(:));
  v=(d(1)+1i*d(2)-z).*exp(-1i*beta(:));f=imag(v);along=real(v);
 end
end
