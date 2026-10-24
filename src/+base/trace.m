function p=trace(q,parent,move,id)
p=zeros(0,4);
while parent(id)>0,p=[move(id,:);p];id=parent(id);end %#ok<AGROW>
end
