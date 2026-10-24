function json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));fwrite(fid,jsonencode(value),'char');
end
