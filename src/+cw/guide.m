function [path,info]=guide(points,other,startXY,goalXY,radius,resolution)
% Deterministic eight-neighbour A*. Geometric guide only, not a vehicle path.
% Point-circle checks at grid nodes; no assertion of swept-vehicle feasibility.
if nargin<6,resolution=.5;end
origin=min(points,[],1);upper=max(points,[],1);sz=ceil((upper-origin)/resolution)+1;
assert(all(sz>0)&&prod(sz)<2e6,'Grid too large.');
occ=false(sz(2),sz(1));
% Exact point-circle occupancy at grid nodes. Full vehicle feasibility is checked separately.
rr=ceil(radius/resolution)+1;[dx,dy]=meshgrid(-rr:rr);dx=dx(:)';dy=dy(:)';
for first=1:1024:size(points,1)
    p=points(first:min(first+1023,size(points,1)),:);base=round((p-origin)/resolution)+1;
    ix=base(:,1)+dx;iy=base(:,2)+dy;
    xx=origin(1)+(ix-1)*resolution;yy=origin(2)+(iy-1)*resolution;
    valid=ix>=1&iy>=1&ix<=sz(1)&iy<=sz(2)&((xx-p(:,1)).^2+(yy-p(:,2)).^2<=radius^2+1e-12);
    occ(sub2ind(size(occ),iy(valid),ix(valid)))=true;
end
if ~isempty(other)
    [xx,yy]=meshgrid(origin(1)+(0:sz(1)-1)*resolution,origin(2)+(0:sz(2)-1)*resolution);
    distance2=inf(size(xx));poly=[other;other(1,:)];
    for k=1:size(other,1)
        delta=poly(k+1,:)-poly(k,:);den=sum(delta.^2);
        t=max(0,min(1,((xx-poly(k,1))*delta(1)+(yy-poly(k,2))*delta(2))/max(den,eps)));
        distance2=min(distance2,(xx-poly(k,1)-t*delta(1)).^2+(yy-poly(k,2)-t*delta(2)).^2);
    end
    occ=occ|inpolygon(xx,yy,other(:,1),other(:,2))|distance2<=radius^2;
end
s=round((startXY-origin)/resolution)+1;g=round((goalXY-origin)/resolution)+1;
path=zeros(0,2);info=struct('status','NO_PATH','expanded',0,'resolution',resolution,'inflation',radius);
if any(s<1|s>sz)||any(g<1|g>sz),info.status='OUTSIDE_GRID';return;end
sid=sub2ind(size(occ),s(2),s(1));gid=sub2ind(size(occ),g(2),g(1));
if occ(sid)||occ(gid),info.status='ENDPOINT_BLOCKED_IN_GUIDE';return;end
N=numel(occ);cost=inf(N,1);parent=zeros(N,1,'uint32');closed=false(N,1);cost(sid)=0;
heapId=zeros(N,1,'uint32');heapKey=inf(N,1);hn=0;push(sid,norm(s-g));
moves=[1 0;1 1;0 1;-1 1;-1 0;-1 -1;0 -1;1 -1];
while hn>0
    [id,key]=pop();if closed(id),continue;end
    [y,x]=ind2sub(size(occ),id);
    if key>cost(id)+hypot(x-g(1),y-g(2))+1e-9,continue;end
    if id==gid,info.status='SUCCESS';break;end
    closed(id)=true;info.expanded=info.expanded+1;
    if info.expanded>=100000,info.status='GUIDE_BUDGET_EXHAUSTED';break;end
    for j=1:8
        nx=x+moves(j,1);ny=y+moves(j,2);
        if nx<1||ny<1||nx>sz(1)||ny>sz(2),continue;end
        ni=sub2ind(size(occ),ny,nx);if occ(ni)||closed(ni),continue;end
        if all(moves(j,:)~=0) && (occ(y,nx)||occ(ny,x)),continue;end
        nc=cost(id)+hypot(moves(j,1),moves(j,2));
        if nc<cost(ni),cost(ni)=nc;parent(ni)=id;push(ni,nc+hypot(nx-g(1),ny-g(2)));end
    end
end
if ~strcmp(info.status,'SUCCESS'),return;end
ids=gid;while ids(end)~=sid,ids(end+1)=double(parent(ids(end)));end %#ok<AGROW>
ids=fliplr(ids);[y,x]=ind2sub(size(occ),ids);path=[x(:)-1 y(:)-1]*resolution+origin;
path(1,:)=startXY;path(end,:)=goalXY;
    function push(id,key)
        hn=hn+1;if hn>numel(heapId),heapId(end+N)=0;heapKey(end+N)=inf;end
        k=hn;while k>1
            par=floor(k/2);if heapKey(par)<key || (heapKey(par)==key && heapId(par)<=id),break;end
            heapId(k)=heapId(par);heapKey(k)=heapKey(par);k=par;
        end
        heapId(k)=id;heapKey(k)=key;
    end
    function [id,key]=pop()
        id=double(heapId(1));key=heapKey(1);lastId=heapId(hn);lastKey=heapKey(hn);hn=hn-1;k=1;
        while 2*k<=hn
            c=2*k;if c<hn && (heapKey(c+1)<heapKey(c)||(heapKey(c+1)==heapKey(c)&&heapId(c+1)<heapId(c))),c=c+1;end
            if lastKey<heapKey(c)||(lastKey==heapKey(c)&&lastId<=heapId(c)),break;end
            heapId(k)=heapId(c);heapKey(k)=heapKey(c);k=c;
        end
        if hn>0,heapId(k)=lastId;heapKey(k)=lastKey;end
    end
end
