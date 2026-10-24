function leg=prefix(q0,gear,side,L,R,Lc)
% Side is physical steering sign. All complete primitives return to kappa=0.
assert(L>=2*Lc);phi=side*gear*(L-Lc)/R;start=q0;start(3)=start(3)+(gear<0)*pi;
p=struct('start',start,'goal',[0 0 0],'segments',cc.turnSegments(phi,R,Lc),'length',L,'bodyOffset',(gear<0)*pi);
p.length=sum(p.segments(:,1));q=cc.sample(p,.2);p.goal=q(end,:);leg=struct('path',p,'gear',gear,'reverseOrder',false);
end
