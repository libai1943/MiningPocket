function p=join(p1,p2)
p=p1;p.goal=p2.goal;p.segments=[p1.segments;p2.segments];p.length=p1.length+p2.length;
end
