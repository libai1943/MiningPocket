function pieces=pieces(result)
if isfield(result.best,'pieces'),pieces=result.best.pieces;else,pieces=cwr.fromBase(result.best);end
end
