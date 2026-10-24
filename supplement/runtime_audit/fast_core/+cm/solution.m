function sol=solution(legs,weight)
if nargin<2,weight=1.35;end
g=cellfun(@(x)x.gear,legs);L=cellfun(@(x)x.path.length,legs);active=L>1e-8;g=g(active);legs=legs(active);L=L(active);
sol=struct('legs',{legs},'cusps',sum(diff(g)~=0),'length',sum(L),'forwardLength',sum(L(g>0)),'reverseLength',sum(L(g<0)),'cost',sum(L(g>0))+weight*sum(L(g<0)));
end
