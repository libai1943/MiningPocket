function h=heuristic(q,target,R)
rc=reedsSheppConnection('MinTurningRadius',R);[~,h]=connect(rc,q,target);h=h(:);
end
