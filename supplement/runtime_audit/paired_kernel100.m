function paired_kernel100
root=fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(root); miningpocket_setup(); audit=fileparts(mfilename('fullpath')); out=fullfile(root,'results','reproduced','runtime_audit'); if ~isfolder(out),mkdir(out);end; addpath(fullfile(audit,'kernel_reference')); addpath(fullfile(audit,'fast_core'),'-begin'); maxNumCompThreads(1);
input=cell(100,1);for i=1:100,load(fullfile(fullfile(root,'benchmark','cases'),sprintf('B%03d.mat',i)),'scene');input{i}=scene;end
s=input{1};a=s.start;b=s.goal;oldcc.connect(a,b,s.truck.radius*[1 1.4],4,1.8*pi);cc.connect(a,b,s.truck.radius*[1 1.4],4,1.8*pi);
rows=struct([]);
for repeat=1:3
 for i=1:100
  s=input{i};a=s.start;b=s.goal+[s.minimumStraight*cos(s.goal(3)),s.minimumStraight*sin(s.goal(3)),0];R=s.truck.radius*[1 1.4];
  if mod(i+repeat,2)
   t=tic;p=oldcc.connect(a,b,R,4,1.8*pi);told=toc(t);t=tic;q=cc.connect(a,b,R,4,1.8*pi);tnew=toc(t);
  else
   t=tic;q=cc.connect(a,b,R,4,1.8*pi);tnew=toc(t);t=tic;p=oldcc.connect(a,b,R,4,1.8*pi);told=toc(t);
  end
  same=numel(p)==numel(q);err=Inf;if same,err=0;if ~isempty(p),err=max(abs(sort(cellfun(@(v)v.length,p))-sort(cellfun(@(v)v.length,q))));end;end
  rows=[rows;struct('id',sprintf('B%03d',i),'repeat',repeat,'oldSeconds',told,'newSeconds',tnew,'oldCandidates',numel(p),'newCandidates',numel(q),'maxLengthDifference',err,'same',same&&err<1e-7)];
 end
end
cb.json(fullfile(out,'paired_kernel100.json'),rows);assert(all([rows.same]));
fprintf('PAIRED KERNEL: 300 pairs; old %.6f s new %.6f s; max length difference %.3g\n',sum([rows.oldSeconds]),sum([rows.newSeconds]),max([rows.maxLengthDifference]));
end
