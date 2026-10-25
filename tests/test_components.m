function test_components()
root=miningpocket_setup();addpath(fullfile(root,'tests'));test_numerics();
for i=[3 58 86]
 d=load(fullfile(root,'benchmark/cases',sprintf('B%03d.mat',i)));
 z=dp.plan(d.scene);assert(z.success);
 selected=cm.solution({struct('path',z.best,'gear',-1,'reverseOrder',true)});
 value=evalcw.score(selected,d.scene.truck);assert(value<=z.coarseJ+1e-8);
 if isfinite(z.fineJ),assert(value<=z.fineJ+1e-8);end
 fprintf('DP %d: coarse %.9f fine %.9f selectedFine=%d\n',i,z.coarseJ,z.fineJ,z.selectedFine);
end
d=load(fullfile(root,'benchmark/cases/B003.mat'));o=cm.options();r=cm.solve(d.scene,o);assert(r.success);val=evalcw.independent(d.scene,r.best);assert(val.passed);
fprintf('BRS case3 J=%.9f, seconds=%.3f\n',evalcw.score(r.best,d.scene.truck),r.seconds);
o=base.options();o.maxSeconds=30;r=base.ccsearch(d.scene,o);assert(r.success);val=evalcw.independent(d.scene,r.best);assert(val.passed);
fprintf('CC-A* case3 J=%.9f, seconds=%.3f\n',evalcw.score(r.best,d.scene.truck),r.seconds);
end
