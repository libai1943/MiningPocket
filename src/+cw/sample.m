function q=sample(path,step)
q=path.start;
for j=1:numel(path.lengths)
 if path.lengths(j)<1e-9,continue;end
 seg=cw.segment(q(end,:),path.lengths(j),path.curvatures(j),step);q=[q;seg(2:end,:)]; %#ok<AGROW>
end
end
