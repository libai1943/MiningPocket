function yes=expired(o)
yes=isfield(o,'clock') && toc(o.clock)>=o.maxSeconds;
end
