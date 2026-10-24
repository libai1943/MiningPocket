function smoke_release
root = fileparts(fileparts(mfilename('fullpath'))); addpath(root);
miningpocket_setup();
rows = run_benchmark({'BRS','HA','BIAGT','GHA'},3);
assert(all([rows.success]),'A portable entry-point smoke test failed.');
for i = [18 55 27]
    r = plan_case(i,'BRS'); assert(r.success,'Branch/continuation smoke test failed.');
end
rows = validate_recorded([3 18 27 55]);
assert(all([rows.passed]));
fprintf('PASS: seven fresh planner calls and sixteen independent stored-result audits.\n');
end
