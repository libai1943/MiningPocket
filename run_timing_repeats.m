function rows=run_timing_repeats()
% Prespecified small timing panel; all results are retained.
root=miningpocket_setup();out=fullfile(root,'results/timing');if ~isfolder(out),mkdir(out);end
ids=[3 18 55];methods={'BRS','HA','BIAGT','GHA','CC-A*'};rows=struct([]);
for rep=1:3
 for i=ids
  d=load(fullfile(root,'benchmark/cases',sprintf('B%03d.mat',i)));scene=d.scene;
  for m=circshift(1:5,[0 mod(rep+i,5)])
   method=methods{m};key=sprintf('%s_%s_r%d.mat',strrep(method,'*','star'),scene.id,rep);target=fullfile(out,key);
   if isfile(target),z=load(target,'row');rows=[rows z.row];continue;end
   clear dp.link
   t=tic;
   switch method
    case 'BRS',r=cm.solve(scene);
    case 'CC-A*',r=base.ccsearch(scene,base.options());
    case 'BIAGT',r=base.biagt(scene,base.options());
    otherwise,r=base.search(scene,method,base.options());
   end
   J=NaN;if r.success,J=evalcw.score(r.best,scene.truck);end
   row=struct('id',scene.id,'method',method,'repetition',rep,'success',r.success,'J',J,'seconds',toc(t));
   save(target,'row','-v7');rows=[rows row];evalcw.writeJson(fullfile(out,'repeats.json'),rows);
   fprintf('REPEAT %d %s %s seconds=%.3f\n',rep,method,scene.id,row.seconds);
  end
 end
end
end

