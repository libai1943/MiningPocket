function writeJson(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));fwrite(fid,unicode2native(jsonencode(value),'UTF-8'),'uint8');
end
