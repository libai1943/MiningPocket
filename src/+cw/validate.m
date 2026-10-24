function report=validate(scene,result)
% Independent full point-array check: no spatial index and no planning collision routine.
assert(result.success);s=result.best;t=scene.truck;step=.08;
if isempty(s.head),f=zeros(0,3);else,f=cw.sample(s.head,step);end
b=flipud(cw.sample(s.tail,step));q=[f;b];
assert(norm(q(1,1:2)-scene.start(1:2))<1e-7);assert(norm(q(end,1:2)-scene.goal(1:2))<1e-7);
assert(abs(wrap(q(1,3)-scene.start(3)))<1e-8);assert(abs(wrap(q(end,3)-scene.goal(3)))<1e-8);
if ~isempty(f),assert(norm(f(end,1:2)-b(1,1:2))<1e-7 && abs(wrap(f(end,3)-b(1,3)))<1e-8);end
k=s.tail.curvatures;if ~isempty(s.head),k=[k s.head.curvatures];end
assert(max(abs(k))<=1/t.radius+1e-12);assert(s.tail.lengths(1)>=scene.minimumStraight-1e-9 && s.tail.curvatures(1)==0);
checkTangent(f,1);checkTangent(b,-1);
w=cw.world(scene,result.options);p=w.points;clearance=inf;
for first=1:48:size(q,1)
 qq=q(first:min(first+47,size(q,1)),:);dx=p(:,1)-qq(:,1)';dy=p(:,2)-qq(:,2)';
 x=dx.*cos(qq(:,3)')+dy.*sin(qq(:,3)');y=-dx.*sin(qq(:,3)')+dy.*cos(qq(:,3)');
 ex=max(max(x-t.front,-t.rear-x),0);ey=max(abs(y)-t.width/2,0);
 clearance=min(clearance,sqrt(min(ex(:).^2+ey(:).^2)));
end
bound=step/2*(1+hypot(t.front,t.width/2)/t.radius);
c=q(:,1:2)+(t.front-t.rear)/2*[cos(q(:,3)) sin(q(:,3))];
support=c*w.normals'+t.length/2*abs([cos(q(:,3)) sin(q(:,3))]*w.normals')+t.width/2*abs([-sin(q(:,3)) cos(q(:,3))]*w.normals');
domainClearance=min(w.limits'-support,[],'all');
assert(domainClearance>=result.options.margin+bound-1e-8,'Path leaves known map domain');
assert(clearance>=result.options.margin+bound-1e-8,'Swept clearance certificate failed');
report=struct('passed',true,'sampleSpacing',step,'minSampledClearance',clearance,'motionBound',bound, ...
 'certifiedDomainClearance',domainClearance-bound,'certifiedPointClearance',clearance-bound,'cuspCount',s.cuspCount,'length',s.forwardLength+s.reverseLength,'maxCurvature',max(abs(k)));
end
function checkTangent(q,gear)
if size(q,1)<2,return;end
d=diff(q(:,1:2));a=q(1:end-1,3)+wrap(diff(q(:,3)))/2;err=wrap(atan2(gear*d(:,2),gear*d(:,1))-a);assert(max(abs(err))<1e-6);
end
function a=wrap(a),a=atan2(sin(a),cos(a));end
