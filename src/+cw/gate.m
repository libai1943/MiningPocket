function g=gate(scene,o)
% Disjoint total partition of well-formed inputs. Never reads a previous result,
% a benchmark ID, or the eventual success of either maneuver planner.
timer=tic;w=cw.world(scene,o);t=scene.truck;delta=atan2(sin(scene.start(3)-scene.goal(3)),cos(scene.start(3)-scene.goal(3)));
[links,linkClearance,lines]=cw.cornerLinks(scene,w,o.margin+o.gateBuffer);
base=[cw.vehicle(scene.start,t);cw.vehicle(scene.goal,t)];h=convhull(base(:,1),base(:,2));base=base(h(1:end-1),:);
[baseFree,baseClearance]=cw.polygonFree(base,w,o.margin+o.gateBuffer);
g=struct('class',3,'name','绕障重定位','method','两端锚点与绕障通道；必要时递归机动','headingDifference',delta, ...
 'smallAngle',abs(delta)<=o.gateAngle+1e-12,'cornerLinks',lines,'linkFree',links,'linkClearance',linkClearance,'all16Free',all(links(:)), ...
 'baseEnvelope',base,'baseEnvelopeFree',baseFree,'baseClearance',baseClearance,'directCertified',false,'localVisible',false,'turningRoom',false,'turnProbes',[], ...
 'witness',[],'witnessCertificate',[],'sweptEnvelope',[],'widening',[],'anchors',zeros(0,4),'visibleEnvelope',[],'visibleAnchor',[],'freeStem',0, ...
 'readsOnlyInput',true,'criterion','C1=A & V16 & Vfilled & W; C2=~C1 & Vanchor & T; C3=~C1 & ~(Vanchor & T)');
if g.smallAngle && g.all16Free && g.baseEnvelopeFree
 [wit,poly,info,cert]=cw.reverseGate(scene,o,w);
 if wit.success,g.directCertified=true;g.witness=wit;g.witnessCertificate=cert;g.sweptEnvelope=poly;g.widening=info;end
end
if g.directCertified
 g.class=1;g.name='直接倒入';g.method='复用已认证的完整倒车曲线';g.reason='SMALL_ANGLE_AND_REVERSE_SWEEP_CERTIFIED';
else
 % Visibility of the whole vehicle corridor to outward terminal-straight anchors.
 % No heading threshold: rejected aligned scenes and opposed scenes can both enter C2.
 for L=scene.minimumStraight:o.stemStep:o.maxStem
  q=cw.segment(scene.goal,L,0,o.fineStep);if cw.collides(q,w,o.fineStep/2),break;end
  g.freeStem=L;a=q(end,:);poly=[cw.vehicle(scene.start,t);cw.vehicle(a,t)];h=convhull(poly(:,1),poly(:,2));poly=poly(h(1:end-1),:);
  [visible,~,~]=cw.polygonFree(poly,w,o.margin+o.gateBuffer);g.anchors(end+1,:)=[a visible];
  if visible && ~g.localVisible,g.localVisible=true;g.visibleAnchor=a;g.visibleEnvelope=poly;end
 end
 [g.turningRoom,g.turnProbes]=cw.turningRoom(scene,o,w);
 if g.localVisible && g.turningRoom,g.class=2;g.name='通道内衔接';g.method='沿末端直线外移锚点，枚举转向曲线与换向姿态';g.reason='NOT_C1_AND_VISIBLE_ANCHOR_AND_TURNING_ROOM';
 else,g.reason='NOT_C1_AND_RESTRICTED_VISIBILITY_OR_TURNING_ROOM';end
end
g.membership=[g.directCertified ~g.directCertified&&g.localVisible&&g.turningRoom ~g.directCertified&&~(g.localVisible&&g.turningRoom)];assert(sum(g.membership)==1 && g.membership(g.class));g.seconds=toc(timer);
end
