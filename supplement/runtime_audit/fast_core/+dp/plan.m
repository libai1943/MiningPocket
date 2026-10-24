function r=plan(scene,o)
if nargin<2,o=dp.options();end
timer=tic;g=dp.graph(scene,o);stemCert=s1.check(scene,g.stem,o.checkStep);
r=struct('success',false,'best',[],'coarse',[],'fine',[],'class',cc.classify(scene),'seconds',0,'options',o,'stemCertificate',stemCert);
if ~stemCert.passed,r.seconds=toc(timer);return;end
r.coarse=dp.search(scene,g,o);r.success=r.coarse.success;
if r.success
 selected=r.coarse;
 if o.refine && g.monotone && numel(g.xs)>2
  gf=dp.graph(scene,o,r.coarse);fineO=o;fineO.maxEvaluations=o.fineEvaluations;
  r.fine=dp.search(scene,gf,fineO,r.coarse);
  if r.fine.success && r.fine.best.length<=selected.best.length+1e-7,selected=r.fine;end
 end
 r.best=cc.join(g.stem,selected.best);r.certificate=s1.check(scene,r.best,o.checkStep);assert(r.certificate.passed);
end
r.seconds=toc(timer);
end
