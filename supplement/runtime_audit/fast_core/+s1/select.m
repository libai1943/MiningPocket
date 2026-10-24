function r=select(scene,paths,mode,step)
% Input may be unsorted. Both modes use exactly the same swept-body validator.
if nargin<3,mode='ordered';end
if nargin<4,step=.04;end
assert(any(strcmp(mode,{'ordered','exhaustive'})));
timer=tic;w=cw.world(scene,cw.defaults());cost=cellfun(@(p)p.length,paths);[~,order]=sortrows([cost(:) (1:numel(paths))'],[1 2]);
r=struct('success',false,'best',[],'winner',0,'checked',0,'seconds',0,'order',order,'status',zeros(1,numel(paths)),'certificates',{cell(1,numel(paths))});
for rank=1:numel(order)
 idx=order(rank);c=s1.check(scene,paths{idx},step,w);r.certificates{idx}=c;r.checked=r.checked+1;
 if c.passed
  r.status(idx)=1;
  if ~r.success,r.success=true;r.best=paths{idx};r.best.certificate=c;r.winner=idx;end
  if strcmp(mode,'ordered'),break;end
 else,r.status(idx)=-1;end
end
r.seconds=toc(timer);
% status: -1 failed certificate; 0 untested; +1 certified safe.
end

