function rows=run_refinement_ablation()
root=miningpocket_setup();rows=struct([]);
for i=1:100
 d=load(fullfile(root,'benchmark/cases',sprintf('B%03d.mat',i)));clear dp.link
 r=dp.plan(d.scene,dp.options());
 row=struct('id',d.scene.id,'success',r.success,'fineAttempted',~isempty(r.fine),'coarseJ',r.coarseJ,'fineJ',r.fineJ,'selectedFine',r.selectedFine,'selectedJ',NaN);
 if r.success
  sol=cm.solution({struct('path',r.best,'gear',-1,'reverseOrder',true)});row.selectedJ=evalcw.score(sol,d.scene.truck);
  assert(row.selectedJ<=r.coarseJ+1e-9);
 end
 rows=[rows row];evalcw.writeJson(fullfile(root,'results/reproduced/refinement100.json'),rows);
end
end
