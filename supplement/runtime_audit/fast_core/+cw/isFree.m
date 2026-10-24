function free=isFree(path,w,o)
% Coarse certified acceptance, exact coarse rejection, fine check only near obstacles.
q=cw.sample(path,o.step);bound=o.step/2*(1+hypot(w.truck.front,w.truck.width/2)/w.truck.radius);
if ~cw.collides(q,w,bound),free=true;return;end
if cw.collides(q,w,0),free=false;return;end
q=cw.sample(path,o.fineStep);bound=o.fineStep/2*(1+hypot(w.truck.front,w.truck.width/2)/w.truck.radius);
free=~cw.collides(q,w,bound);
end
