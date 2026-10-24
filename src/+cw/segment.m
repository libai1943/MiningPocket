function q=segment(q0,length,curvature,step)
n=max(1,ceil(length/step));s=linspace(0,length,n+1)';a=q0(3);
if abs(curvature)<1e-12
 q=[q0(1)+s*cos(a),q0(2)+s*sin(a),repmat(a,size(s))];
else
 th=a+curvature*s;q=[q0(1)+(sin(th)-sin(a))/curvature,q0(2)-(cos(th)-cos(a))/curvature,th];
end
end
