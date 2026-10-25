function solutions=ccwordShots(start,goal,scene,w,o,clock)
% RS words supply cusp poses; the shared CC connector supplies every motion.
[words,~]=base.connections(start,goal,scene.truck.radius,o,'rs');
solutions={};
for j=1:numel(words)
 if toc(clock)>=o.maxSeconds,break;end
 v=words{j};q=start;poses=q;gears=[];last=0;
 for k=1:size(v,1)
  qq=base.primitive(q,v(k,1),v(k,2),v(k,4),v(k,1));q=qq(end,:);
  if k==size(v,1)||v(k+1,4)~=v(k,4),poses(end+1,:)=q;gears(end+1)=v(k,4);end
 end
 if sum(diff(gears)~=0)>3,continue;end
 legs={};valid=true;
 for k=1:numel(gears)
  a=poses(k,:);b=poses(k+1,:);gear=gears(k);
  if gear<0,pp=cc.connect(b,a,scene.truck.radius*[1 1.4],[1 4],1.8*pi);
  else,pp=cc.connect(a,b,scene.truck.radius*[1 1.4],[1 4],1.8*pi);end
  chosen=[];
  for l=1:numel(pp)
   cert=s1.check(scene,pp{l},.04,w);if cert.passed,chosen=pp{l};break;end
  end
  if isempty(chosen),valid=false;break;end
  legs{end+1}=struct('path',chosen,'gear',gear,'reverseOrder',gear<0);
 end
 if valid,solutions{end+1}=legs;if numel(solutions)>=3,break;end;end
end
end
