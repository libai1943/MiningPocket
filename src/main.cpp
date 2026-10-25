#include "../include/core.hpp"
namespace mp {
json runQuery(const Scene &s, const Params &p, const std::string &method, const Options &o) {
    Context context(p.deadline, p.reserve);
    context.original = &s;
    ctx = &context;
    Result r;
    try {
        {
            Scope scope("query");
            World w(s, p);
            deadline();
            if (method == "BRS")
                r = brs(s, w, o);
            else if (method == "BIAGT")
                r = biagt(s, w, o);
            else if (method == "CC-A*")
                r = ccsearch(s, w, o);
            else if (method == "HA" || method == "GHA")
                r = search(s, w, method, o);
            else
                throw std::runtime_error("Unknown method");
        }
    } catch (Timeout &) {
        r.reason = "timeout";
    } catch (...) {
        ctx = nullptr;
        throw;
    }
    r.seconds = context.elapsed();
    ctx = nullptr;
    if (r.seconds > p.deadline) {
        r.success = false;
        r.reason = "deadline_exceeded";
    }
    json stats;
    for (auto &[k, v] : context.stats)
        stats[k] = {{"seconds", v.seconds}, {"calls", v.calls}};
    json j = {{"case", s.id},
              {"method", method},
              {"success", r.success},
              {"reason", r.reason},
              {"planning_seconds", r.seconds},
              {"first_feasible_seconds", r.first >= 0 ? json(r.first) : json(nullptr)},
              {"branch", r.branch},
              {"expanded", r.expanded},
              {"candidates", r.candidates},
              {"prefix_depth", r.prefixDepth},
              {"deadline_seconds", p.deadline},
              {"deadline_checks", context.checks},
              {"selection_limit", o.candidateLimit},
              {"selection_seconds", o.selectionSeconds},
              {"stages", stats}};
    if (r.success) {
        j["path"] = jpath(r.pieces);
        j["execution_seconds"] = r.timing.total;
        j["moving_seconds"] = r.timing.moving;
        j["dwell_seconds"] = r.timing.dwell;
        j["extra_stops"] = r.timing.stops;
        j["cusps"] = r.timing.cusps;
        j["J"] = r.timing.J;
        j["length"] = r.timing.L;
    }
    return j;
}
} // namespace mp
using namespace mp;
std::string caseFile(const std::string &root, int i) {
    std::ostringstream ss;
    ss << root << "/benchmark/case_" << std::setw(3) << std::setfill('0') << i << ".json";
    return ss.str();
}
int main(int argc, char **argv) {
    try {
        if (argc < 3)
            throw std::runtime_error(
                "Usage: miningpocket run|parity|selftest ROOT [--case N|--all] [--method NAME] "
                "[--budget SECONDS] [--out PATH] [--refine] [--policy 2|3] [--no-prefix]");
        std::string command = argv[1], root = argv[2], method = "BRS", out;
        int first = 1, last = 1;
        Params p = readParams(root + "/benchmark/parameters.json");
        Options o;
        {
            std::ifstream configFile(root + "/benchmark/parameters.json");
            json config;
            configFile >> config;
            o.candidateLimit = config.value("brs_candidate_limit", 1);
            o.selectionSeconds = config.value("brs_selection_seconds", 0.);
        }
        for (int i = 3; i < argc; i++) {
            std::string a = argv[i];
            if (a == "--case") {
                first = last = std::stoi(argv[++i]);
            } else if (a == "--all") {
                first = 1;
                last = 100;
            } else if (a == "--method")
                method = argv[++i];
            else if (a == "--budget")
                p.deadline = std::stod(argv[++i]);
            else if (a == "--out")
                out = argv[++i];
            else if (a == "--refine")
                o.refine = true;
            else if (a == "--candidates")
                o.candidateLimit = std::stoi(argv[++i]);
            else if (a == "--improve-seconds")
                o.selectionSeconds = std::stod(argv[++i]);
            else if (a == "--first-only")
                o.diagnosticFirstOnly = true;
            else if (a == "--fixed-order")
                o.fixedOrder = true;
            else if (a == "--policy")
                o.brsPolicy = std::stoi(argv[++i]);
            else if (a == "--no-prefix")
                o.depth = 0;
            else
                throw std::runtime_error("Unknown option " + a);
        }
        if (o.diagnosticFirstOnly) {
            o.candidateLimit = 1;
            o.selectionSeconds = 0;
        }
        if (command == "run") {
            std::ofstream file;
            if (!out.empty())
                file.open(out);
            for (int id = first; id <= last; id++) {
                Scene s = readScene(caseFile(root, id));
                auto j = runQuery(s, p, method, o);
                std::cout << j.dump() << std::endl;
                if (file)
                    file << j.dump() << std::endl;
            }
            return 0;
        }
        if (command == "validate-results") {
            std::ifstream f(out);
            std::string line;
            int count = 0, failed = 0;
            json details = json::array();
            while (std::getline(f, line)) {
                if (line.empty())
                    continue;
                json rec = json::parse(line);
                if (!rec["success"].get<bool>())
                    continue;
                Scene s = readScene(caseFile(root, rec["case"]));
                World w(s, p);
                Path v = path(rec["path"]);
                Timing t = profile(v, s.truck, p);
                bool ok = validate(s, w, v, false, p.verifyStep) && t.valid &&
                          std::abs(t.total - rec["execution_seconds"].get<double>()) < 1e-7 &&
                          std::abs(t.J - rec["J"].get<double>()) < 1e-7;
                failed += !ok;
                count++;
                if (!ok)
                    details.push_back({{"case", rec["case"]}, {"method", rec["method"]}});
            }
            std::cout
                << json({{"validated", count}, {"failed", failed}, {"details", details}}).dump()
                << std::endl;
            return failed ? 2 : 0;
        }
        if (command == "inspect") {
            std::ifstream f(out);
            std::string line;
            while (std::getline(f, line)) {
                if (line.empty())
                    continue;
                json rec = json::parse(line);
                if (rec["case"] != first || rec["method"] != method || !rec["success"].get<bool>())
                    continue;
                Scene s = readScene(caseFile(root, first));
                Path v = path(rec["path"]);
                Timing t = profile(v, s.truck, p, true);
                json poses = json::array();
                Pose q = s.start;
                double d = 0;
                for (auto &z : v) {
                    int n = std::max(1, int(std::ceil(z.L / .1)));
                    for (int i = 0; i < n; i++) {
                        auto x = advance(q, z, z.L * i / n);
                        poses.push_back(
                            {d + z.L * i / n, x.x, x.y, x.t, z.gear, z.k0 + (z.k1 - z.k0) * i / n});
                    }
                    q = advance(q, z);
                    d += z.L;
                }
                poses.push_back({d, q.x, q.y, q.t, v.back().gear, v.back().k1});
                std::cout << json({{"case", first},
                                   {"method", method},
                                   {"samples", poses},
                                   {"events", t.events},
                                   {"profile", t.profile},
                                   {"path", rec["path"]}})
                                 .dump()
                          << std::endl;
                return 0;
            }
            throw std::runtime_error("Record not found");
        }
        if (command == "parity") {
            std::ifstream f(root + "/tests/reference_paths.json");
            json fixtures;
            f >> fixtures;
            json rows = json::array();
            double maxTime = 0, maxEnd = 0;
            int failed = 0, lastId = 0;
            Scene s;
            std::unique_ptr<World> w;
            for (auto &ref : fixtures) {
                int id = ref["case"];
                if (id != lastId) {
                    s = readScene(caseFile(root, id));
                    w = std::make_unique<World>(s, p);
                    lastId = id;
                }
                Path v = path(ref["pieces"]);
                Timing t = profile(v, s.truck, p);
                Pose e = endpoint(s.start, v);
                double de = distance(e, s.goal),
                       dt = std::abs(t.total - ref["expected"]["totalTime"].get<double>());
                bool collisionOK = validate(s, *w, v, false, p.verifyStep),
                     stops = t.stops == ref["expected"]["internalStops"].get<int>(),
                     ok = t.valid && dt < 1e-7 && stops && collisionOK;
                maxTime = std::max(maxTime, dt);
                maxEnd = std::max(maxEnd, de);
                failed += !ok;
                rows.push_back({{"case", id},
                                {"method", ref["method"]},
                                {"ok", ok},
                                {"endpoint_error", de},
                                {"time_error", dt},
                                {"computed_time", t.total},
                                {"expected_time", ref["expected"]["totalTime"]},
                                {"stops_match", stops},
                                {"collision_valid", collisionOK},
                                {"dynamics_valid", t.valid}});
            }
            json report = {{"count", rows.size()},
                           {"failed", failed},
                           {"max_time_error", maxTime},
                           {"max_endpoint_error", maxEnd},
                           {"rows", rows}};
            if (!out.empty()) {
                std::ofstream f(out);
                f << report.dump(2);
            }
            std::cout << json({{"count", rows.size()},
                               {"failed", failed},
                               {"max_time_error", maxTime},
                               {"max_endpoint_error", maxEnd}})
                             .dump()
                      << std::endl;
            return failed ? 2 : 0;
        }
        if (command == "selftest") {
            std::mt19937_64 rng(1729);
            std::uniform_real_distribution<double> d(-80, 80), a(-pi, pi);
            int total = 0, fail = 0;
            double maxEndpoint = 0;
            for (int i = 0; i < 1000; i++) {
                Pose s{d(rng), d(rng), a(rng)}, g{d(rng), d(rng), a(rng)};
                auto rs = rsWords(s, g, 15), du = dubinsWords(s, g, 15);
                if (rs.empty() || du.empty())
                    fail++;
                for (auto &ps : {rs, du})
                    for (auto &v : ps) {
                        auto e = endpoint(s, v);
                        maxEndpoint = std::max(maxEndpoint, distance(e, g));
                        fail += distance(e, g) > 1e-7 || std::abs(wrap(e.t - g.t)) > 1e-8;
                        total++;
                    }
                auto cc = connect(s, g, {15, 21}, {1, 4});
                for (auto &v : cc) {
                    auto e = endpoint(s, v);
                    fail += distance(e, g) > 1e-5 || std::abs(wrap(e.t - g.t)) > 1e-7;
                    total++;
                }
            }
            Truck truck;
            Path straight = {{40, 0, 0, -1}}, tiny = {{20, 0, 0, -1}};
            auto t1 = invert(turn(1e-12, 15, 4)), t2 = invert(turn(-1e-12, 15, 4));
            tiny = cat(tiny, t1);
            tiny = cat(tiny, t2);
            tiny.push_back({20 - length(t1) - length(t2), 0, 0, -1});
            double delta = profile(tiny, truck, p).total - profile(straight, truck, p).total;
            fail += std::abs(delta) > 1e-7;
            std::cout << json({{"paths", total},
                               {"failed", fail},
                               {"maximum_endpoint_error", maxEndpoint},
                               {"near_zero_time_delta", delta}})
                             .dump()
                      << std::endl;
            return fail ? 2 : 0;
        }
        throw std::runtime_error("Unknown command");
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
