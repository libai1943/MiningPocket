function p=transform(p,origin,angle)
R=[cos(angle) -sin(angle);sin(angle) cos(angle)];
p.start=[p.start(1:2)*R'+origin,p.start(3)+angle];p.goal=[p.goal(1:2)*R'+origin,p.goal(3)+angle];
if isfield(p,'middleHeading'),p.middleHeading=p.middleHeading+angle;end
end
