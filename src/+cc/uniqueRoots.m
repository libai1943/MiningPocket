function r=uniqueRoots(values,tol)
% Keep exact preferred roots; proximity tests never rewrite a root value.
r=[];for v=values(:)',if isfinite(v)&&all(abs(v-r)>tol),r(end+1)=v;end;end
r=sort(r);
end
