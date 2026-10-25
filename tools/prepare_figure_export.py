from pathlib import Path
r=Path(__file__).resolve().parents[1];s=(r/'src/brs.cpp').read_text()
s=s.replace('valid.push_back(!w.collision(s.start));','valid.push_back(!w.collision(s.start));\n    dagTrace["states"]=json::array(); for(size_t j=0;j<poses.size();j++)dagTrace["states"].push_back({{"q",jpose(world(poses[j]))},{"valid",bool(valid[j])}});')
s=s.replace('if (w.check(world(poses[e.u]), e.paths[e.index], 0, true))','bool traceSafe=w.check(world(poses[e.u]), e.paths[e.index], 0, true);\n            dagTrace["edges"].push_back({{"start",jpose(world(poses[e.u]))},{"path",jpath(e.paths[e.index])},{"safe",traceSafe}});\n            if (traceSafe)')
(r/'build/brs_trace.inc').write_text(s)
print('Generated an observation-only copy of the frozen DAG routine for figure export')
