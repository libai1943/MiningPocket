function rows=run_component_experiments(ids)
% Components share geometry, collision checks, curve family and limits.
if nargin<1,ids=1:100;end
root=miningpocket_setup();out=fullfile(root,'results/components');if ~isfolder(out),mkdir(out);end
rows=struct([]);names={'goal_side','start_side','first_feasible'};
for i=ids
 d=load(fullfile(root,'benchmark/cases',sprintf('B%03d.mat',i)));scene=d.scene;
 for m=1:numel(names)
  target=fullfile(out,sprintf('%s_%s.mat',names{m},scene.id));
  if isfile(target),z=load(target,'row');rows=[rows z.row];continue;end
  o=cm.options();
  switch names{m}
   case 'goal_side',o.enabledBranches=2;
   case 'start_side',o.enabledBranches=3;
   case 'first_feasible',o.branchPolicy='first';o.feasibleLimit=1;
  end
  clear dp.link
  t=tic;r=[];row=struct('id',scene.id,'component',names{m},'success',false,'J',NaN,'seconds',NaN,'candidates',0,'recursive',false,'error','');
  try
   r=cm.solve(scene,o);row.recursive=~r.root.success;
   if r.success,[row.J,e]=evalcw.score(r.best,scene.truck);end
   row.seconds=toc(t);
   if r.success,v=evalcw.independent(scene,r.best);row.success=e.valid&&v.passed;end
   for k=1:numel(r.root.trials),row.candidates=row.candidates+r.root.trials{k}.candidateCount;end
   for k=1:numel(r.nodes),if r.nodes{k}.tested,tt=r.nodes{k}.local.trials;for j=1:numel(tt),row.candidates=row.candidates+tt{j}.candidateCount;end;end;end
  catch err
   row.seconds=toc(t);row.error=getReport(err,'extended','hyperlinks','off');
  end
  save(target,'r','row','-v7');rows=[rows row];evalcw.writeJson(fullfile(out,'components100.json'),rows);
  fprintf('%s %s valid=%d J=%.3f seconds=%.3f\n',names{m},scene.id,row.success,row.J,row.seconds);
 end
end
if numel(rows)==300
 run_timing_repeats();
 export_paper_data();
 validate_holdout_geometry();
 addpath(fullfile(root,'tests'));test_numerics();
 fid=fopen(fullfile(root,'results/validated/completed.flag'),'w');fprintf(fid,'%s',datestr(now,30));fclose(fid);
end
end
