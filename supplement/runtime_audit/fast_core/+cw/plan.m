function result=plan(scene,o)
if nargin<2,o=cw.defaults();end
timer=tic;g=cw.gate(scene,o);
if g.class==1
 result=g.witness;primarySuccess=true;result.preBridgeSuccess=true;
elseif strcmp(o.mode,'adaptive')
 primary=cw.primary(scene,o,g);primarySuccess=primary.success;
 % The class-specific constructor proposes candidates. The common kernel preserves
 % the previous representation and supplies a cost bound, preventing quality loss.
 common=cw.corePlan(scene,o);
 seedGate=g;seedGate.class=cw.headingSector(scene);seedGate.witness=[];
 if seedGate.class~=g.class
  alternative=cw.primary(scene,o,seedGate);
  if alternative.success && (~common.success||alternative.best.cost<common.best.cost),common=alternative;end
 end
 if primary.success && (~common.success || primary.best.cost<=common.best.cost)
  result=primary;
 else
  result=common;result.primaryTrace=primary;
 end
 result.preBridgeSuccess=result.success;
 if ~result.success && strcmp(result.status,'NO_CONNECTION'),result=cw.bridge(scene,o,result);end
else
 result=cw.corePlan(scene,o);primarySuccess=result.success;result.preBridgeSuccess=result.success;
end
result.gate=g;result.sceneClass=g.class;result.className=g.name;
result.primaryConstruction=g.method;result.primarySuccess=primarySuccess;
result.seconds=toc(timer);
end
