function manifest=make_holdout()
% Predeclared independent polygonal layouts; no planner is invoked.
root=miningpocket_setup();out=fullfile(root,'benchmark/holdout');if ~isfolder(out),mkdir(out);end
rng(20261025,'twister');d=load(fullfile(root,'benchmark/cases/B003.mat'));truck=d.scene.truck;manifest=struct([]);
for i=1:12
 width=11+4*rand;depth=19+10*rand;outer=76+20*rand;top=96+12*rand;
 p=[-width/2 -7;width/2 -7;width/2 depth;outer*.30 depth+7;outer/2 depth+15;outer/2 top;-outer/2 top;-outer/2 depth+12;-outer*.26 depth+4;-width/2 depth];
 % Irregular shoulders vary as geometry, rather than duplicating a map.
 p(4,2)=p(4,2)+4*rand;p(9,2)=p(9,2)-3*rand;
 boundary=[];maxGap=0;
 for j=1:size(p,1)
  a=p(j,:);b=p(mod(j,size(p,1))+1,:);n=ceil(norm(b-a)/.1);f=(0:n-1)'/n;
  boundary=[boundary;a+(b-a).*f];maxGap=max(maxGap,norm(b-a)/n);
 end
 center=[13+3*rand,depth+7];radius=4+2*rand;n=ceil(2*pi*radius/.1);ang=(0:n-1)'*2*pi/n;
 boundary=[boundary;center+radius*[cos(ang) sin(ang)]];
 circleGap=radius*pi/n;mapError=max(maxGap/2,circleGap);
 pk=[-13-4*rand,depth+8,pi/2];
 sx=[-18 0 18];sh=[-25 65 155];start=[sx(mod(i-1,3)+1),top-18,pi/2+deg2rad(sh(mod(floor((i-1)/3),3)+1))];
 scene=struct('id',sprintf('H%03d',i),'originalId',sprintf('H%03d',i),'mapGroup',sprintf('H%02d',i),'units','metres_radians','boundary',boundary,'reference',zeros(0,3),'goal',[0 0 pi/2],'start',start,'truck',truck,'parkedTruck',pk,'minimumStraight',12.54,'sourceId',sprintf('procedural_seed20261025_map%02d',i),'taskDefinition','Predeclared independent polygonal loading pocket','provenance',struct('kind','independent_constructed_geometry','seed',20261025,'index',i),'workspacePolygon',p,'exclusionCircle',[center radius],'boundaryCoverageRadius',mapError);
 w=cw.world(scene,cw.defaults());assert(~cw.collides([scene.start;scene.goal],w,0));
 save(fullfile(out,[scene.id '.mat']),'scene','-v7');
 row=struct('id',scene.id,'seed',20261025,'mapGroup',scene.mapGroup,'width',width,'depth',depth,'outerWidth',outer,'top',top,'boundaryCoverageRadius',mapError);manifest=[manifest row];
end
evalcw.writeJson(fullfile(root,'benchmark/holdout_manifest.json'),manifest);
end
