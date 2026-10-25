function rows=test_numerics()
root=miningpocket_setup();d=load(fullfile(root,'benchmark/cases/B003.mat'));truck=d.scene.truck;
o=evalcw.options();straight=evalcw.profile([40 0 0 -1],truck,o);
rows=struct([]);
for phi=[0 1e-14 1e-12 1e-10 1e-8 1e-6 .01]
 [z,L,par]=cc.turn(phi,15,4);[~,L2]=cc.turn(-phi,15,4);
 seg=[20-(L+L2)/2 0 0;cc.turnSegments(phi,15,4);cc.turnSegments(-phi,15,4);20-(L+L2)/2 0 0];seg=seg(seg(:,1)>0,:);
 p=[seg(:,1),seg(:,2),seg(:,2)+seg(:,1).*seg(:,3),-ones(size(seg,1),1)];e=evalcw.profile(p,truck,o);
 assert(e.valid && abs(sum(seg(:,1))-40)<1e-9);
 if phi<=1e-8
  testPath=struct('start',[0 0 0],'segments',seg);qe=cc.endpoint(testPath);
  assert(norm(qe(1:2)-[40 0])<1e-5 && abs(qe(3))<1e-7);
 end
 row=struct('angle',phi,'time',e.totalTime,'difference',e.totalTime-straight.totalTime,'sharpness',par(4),'turnLength',L,'displacement',[real(z),imag(z)]);
 rows=[rows row];
end
assert(max(abs([rows(1:5).difference]))<.01);
for theta=[0 .123456789123 pi/3 -1.22]
 a=[0 0 theta];b=[40*cos(theta),40*sin(theta),theta];
 pp=cc.connect(a,b,15,4);assert(~isempty(pp));p=pp{1};qe=cc.endpoint(p);
 assert(norm(qe(1:2)-b(1:2))<1e-8 && abs(qe(3)-b(3))<1e-10);
 e=evalcw.profile([p.segments(:,1),p.segments(:,2),p.segments(:,2)+p.segments(:,1).*p.segments(:,3),-ones(size(p.segments,1),1)],truck,o);
 assert(abs(e.totalTime-straight.totalTime)<1e-6);
end
evalcw.writeJson(fullfile(root,'results/reproduced/numerical_regression.json'),rows);
disp(rows);disp('Near-zero and exact-heading tests passed.');
end
