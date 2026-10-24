function r=plan(scene,o)
if nargin<2,o=s1.options();end
[paths,meta]=s1.pool(scene,o);r=s1.select(scene,paths,'ordered',o.checkStep);
r.paths=paths;r.meta=meta;r.class=cc.classify(scene);r.totalSeconds=meta.generationSeconds+r.seconds;
end
