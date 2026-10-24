classdef Heap < handle
 properties
  keys=zeros(4096,1);ids=zeros(4096,1);n=0;
 end
 methods
  function push(o,key,id)
   o.n=o.n+1;i=o.n;if i>numel(o.keys),o.keys(end+4096)=0;o.ids(end+4096)=0;end
   while i>1,p=floor(i/2);if o.keys(p)<=key,break;end;o.keys(i)=o.keys(p);o.ids(i)=o.ids(p);i=p;end
   o.keys(i)=key;o.ids(i)=id;
  end
  function [id,key]=pop(o)
   if o.n==0,id=0;key=Inf;return;end
   id=o.ids(1);key=o.keys(1);k=o.keys(o.n);v=o.ids(o.n);o.n=o.n-1;i=1;
   while 2*i<=o.n,c=2*i;if c<o.n&&o.keys(c+1)<o.keys(c),c=c+1;end;if o.keys(c)>=k,break;end;o.keys(i)=o.keys(c);o.ids(i)=o.ids(c);i=c;end
   o.keys(i)=k;o.ids(i)=v;
  end
  function [id,key]=peek(o)
   if o.n==0,id=0;key=Inf;else,id=o.ids(1);key=o.keys(1);end
  end
 end
end
