function r=plan(scene,o)
if nargin<2,o=dp.options();end
timer=tic;g=dp.graph(scene,o);stemCert=s1.check(scene,g.stem,o.checkStep);
r=struct('success',false,'best',[],'coarse',[],'fine',[],'class',cc.classify(scene),'seconds',0,'options',o,'stemCertificate',stemCert,'coarseJ',NaN,'fineJ',NaN,'selectedFine',false);
if ~stemCert.passed,r.seconds=toc(timer);return;end
r.coarse=dp.search(scene,g,o);r.success=r.coarse.success;
if r.success
 selected=r.coarse;r.coarseJ=score(selected.best);
 if o.refine && g.monotone && numel(g.xs)>2
  gf=dp.graph(scene,o,r.coarse);fineO=o;fineO.maxEvaluations=o.fineEvaluations;
  r.fine=dp.search(scene,gf,fineO,r.coarse);
  if r.fine.success
   r.fineJ=score(r.fine.best);
   if r.fineJ<r.coarseJ-1e-9,selected=r.fine;r.selectedFine=true;end
  end
 end
 r.best=cc.join(g.stem,selected.best);r.certificate=s1.check(scene,r.best,o.checkStep);assert(r.certificate.passed);
end
r.seconds=toc(timer);
 function J=score(path)
  whole=cc.join(g.stem,path);sol=cm.solution({struct('path',whole,'gear',-1,'reverseOrder',true)});
  J=evalcw.score(sol,scene.truck);
 end
end
