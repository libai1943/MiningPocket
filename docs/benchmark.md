# Fixed 100-task benchmark

Coordinates are local planar metres; heading angles are radians. The rear-axle centre is the reference point. The packaged scene structure in each MATLAB file contains boundary, start, goal, truck, parkedTruck, and minimumStraight. The boundary field stores the fixed sampled obstacle representation; the planner also enforces its outer convex hull. A stationary truck is represented by its sampled rectangular footprint. All methods use the same geometric inputs and full-body collision rules.

manifest.json maps public Case #1--#100 to internal B001--B100, source IDs, map groups, and construction categories. The benchmark contains 91 tasks placed on reconstructed mine-map geometry, eight constructed environments, and one task with a corrected start. They derive from six map groups and are not 100 independently surveyed mine sites. Starting and goal poses are controlled task definitions. The benchmark is a curated feasible set, not a random estimate of the occurrence of infeasible loading operations.

The class field describes source task organization. The planner computes the heading group from actual input poses; it never reads that label to select a successful recorded branch. Heading groups prioritize branches and do not uniquely identify the branch that will succeed.

The vehicle is 10.40 m long and 4.90 m wide, with rear-axle offsets 8.32 m forward and 2.08 m rearward, wheelbase 6.22 m, and minimum turning radius 15 m. Planning adds a 0.50 m buffer at the front, rear, and both sides. The physical body and planning contour are distinct. The compulsory final reverse straight has length 12.54 m. Inputs and all recorded results are fingerprinted in qa/INPUT_RESULT_SHA256.json.

Do not alter input geometry to rescue a failed rerun. Report any failure, runtime limit, and changed implementation or platform explicitly.
