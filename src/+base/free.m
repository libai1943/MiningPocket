function ok=free(q,w,k,step)
rho=hypot(max(w.truck.front,w.truck.rear)+w.margin,w.truck.width/2+w.margin);extra=step/2*(1+rho*abs(k))+1e-5;
subset=unique([1:25:size(q,1),size(q,1)]);if cw.collides(q(subset,:),w,0),ok=false;else,ok=~cw.collides(q,w,extra);end
end
