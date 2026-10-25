#include "../include/core.hpp"
namespace mp {
struct SelectionBudget {
    double saved;
    SelectionBudget() : saved(ctx->stopAt) {}
    ~SelectionBudget() {
        ctx->stopAt = saved;
    }
    void start(double seconds) {
        ctx->stopAt = std::min(saved, ctx->elapsed() + seconds);
    }
};
struct PrefixContext {
    Path saved;
    PrefixContext(const Path &p) : saved(std::move(ctx->retainedPrefix)) {
        ctx->retainedPrefix = p;
    }
    ~PrefixContext() {
        ctx->retainedPrefix = std::move(saved);
    }
};
static Pose anchor(const Scene &s) {
    return advance(s.goal, {s.tail, 0, 0, 1});
}
static Path tail(const Scene &s) {
    return {{s.tail, 0, 0, -1}};
}
static std::vector<double> scales(double R, const std::vector<double> &f) {
    std::vector<double> v;
    for (double x : f)
        v.push_back(R * x);
    return v;
}
static bool direct(const Scene &s, World &w, int gear, Result &r, const Options &o) {
    Scope scope("direct_connection");
    SelectionBudget budget;
    try {
        Pose A = anchor(s);
        auto candidates = gear < 0 ? connect(A, s.start, scales(s.truck.R, {1, 1.3, 1.7}), {4, 8})
                                   : connect(s.start, A, scales(s.truck.R, {1, 1.3, 1.7}), {4, 8});
        for (auto &p : candidates) {
            deadline();
            Path exec = gear < 0 ? invert(p) : p;
            if (!w.check(s.start, exec, 0, true))
                continue;
            Path all = cat(exec, tail(s));
            if (accept(s, w, all, r, gear < 0 ? "I-direct" : "II-direct", true)) {
                if (gear < 0 || r.candidates >= o.candidateLimit || o.selectionSeconds <= 0)
                    return true;
                if (r.candidates == 1)
                    budget.start(o.selectionSeconds);
            }
        }
    } catch (Timeout &) {
        if (!r.success)
            throw;
    }
    return r.success;
}
struct DAGResult {
    bool success = false;
    Path curve;
    std::vector<Pose> route;
};
static DAGResult dag(const Scene &s, World &w, const DAGResult *coarse = nullptr) {
    Scope scope(coarse ? "local_refinement" : "layered_search");
    Pose A = anchor(s);
    double D = distance(A, s.start), axis = std::atan2(s.start.y - A.y, s.start.x - A.x);
    Pose qa{0, 0, wrap(A.t - axis)}, qb{D, 0, wrap(s.start.t - axis)};
    DAGResult result;
    if (D <= 6 || std::abs(qa.t) >= 85 * pi / 180 || std::abs(qb.t) >= 85 * pi / 180)
        return result;
    auto world = [&](Pose q) {
        return Pose{A.x + q.x * std::cos(axis) - q.y * std::sin(axis),
                    A.y + q.x * std::sin(axis) + q.y * std::cos(axis), q.t + axis};
    };
    int n = std::max(1, int(std::ceil(D / 22))), layers = coarse ? 2 * n : n;
    std::vector<Pose> poses{qa};
    std::vector<std::vector<int>> groups(layers + 1);
    groups[0] = {0};
    std::vector<bool> valid{true};
    std::vector<Pose> sample;
    if (coarse) {
        Pose q = A;
        sample.push_back(qa);
        for (auto &p : coarse->curve) {
            int k = std::max(1, int(std::ceil(p.L / .25)));
            for (int j = 1; j <= k; j++) {
                Pose z = advance(q, p, p.L * j / k);
                double dx = z.x - A.x, dy = z.y - A.y;
                sample.push_back({dx * std::cos(axis) + dy * std::sin(axis),
                                  -dx * std::sin(axis) + dy * std::cos(axis), wrap(z.t - axis)});
            }
            q = advance(q, p);
        }
        std::stable_sort(sample.begin(), sample.end(), [](Pose a, Pose b) { return a.x < b.x; });
    }
    for (int l = 1; l < layers; l++) {
        deadline();
        double x = D * l / layers, cy = 0, ct = 0;
        std::vector<double> ys, ths;
        if (coarse) {
            auto it = std::lower_bound(sample.begin(), sample.end(), x,
                                       [](Pose a, double b) { return a.x < b; });
            int j = std::clamp(int(it - sample.begin()), 1, int(sample.size()) - 1);
            double den = sample[j].x - sample[j - 1].x,
                   u = den > 1e-12 ? (x - sample[j - 1].x) / den : 0;
            cy = sample[j - 1].y + u * (sample[j].y - sample[j - 1].y);
            ct = sample[j - 1].t + u * (sample[j].t - sample[j - 1].t);
            for (double y : {-3., -1.5, 0., 1.5, 3.})
                ys.push_back(cy + y);
            for (double t : {-15., -7.5, 0., 7.5, 15.})
                ths.push_back(ct + t * pi / 180);
        } else {
            for (int y = -24; y <= 24; y += 6)
                ys.push_back(y);
            for (int t = -30; t <= 30; t += 15)
                ths.push_back(t * pi / 180);
        }
        std::vector<Pose> cand;
        for (double t : ths)
            for (double y : ys)
                if (std::abs(t) < 85 * pi / 180)
                    cand.push_back({x, y, t});
        if (coarse)
            for (auto q : coarse->route)
                if (std::abs(q.x - x) < 1e-5)
                    cand.push_back(q);
        for (auto q : cand) {
            bool dup = false;
            for (int i : groups[l])
                if (distance(q, poses[i]) < 1e-8 && std::abs(q.t - poses[i].t) < 1e-8) {
                    dup = true;
                    break;
                }
            if (dup)
                continue;
            groups[l].push_back(int(poses.size()));
            poses.push_back(q);
            valid.push_back(!w.collision(world(q)));
        }
    }
    groups.back().push_back(int(poses.size()));
    poses.push_back(qb);
    valid.push_back(!w.collision(s.start));
    struct Edge {
        int u, v, state = 0, index = 0;
        double weight;
        bool direct;
        std::vector<Path> paths;
    };
    std::vector<Edge> edges;
    std::vector<std::vector<int>> incoming(poses.size());
    std::set<std::pair<int, int>> pairs;
    auto add = [&](int u, int v, bool direct) {
        if (!pairs.insert({u, v}).second)
            return;
        incoming[v].push_back(int(edges.size()));
        edges.push_back({u, v, 0, 0, distance(poses[u], poses[v]), direct, {}});
    };
    add(0, int(poses.size()) - 1, true);
    for (int l = 0; l < layers; l++)
        for (int jump = 1; jump <= (coarse ? 2 : 1) && l + jump <= layers; jump++)
            for (int u : groups[l]) {
                if (!valid[u])
                    continue;
                for (int v : groups[l + jump]) {
                    if (!valid[v])
                        continue;
                    double dx = poses[v].x - poses[u].x;
                    if (std::abs(poses[v].y - poses[u].y) > std::max(6., .65 * dx) ||
                        std::abs(poses[v].t - poses[u].t) > 65 * pi / 180)
                        continue;
                    add(u, v, false);
                }
            }
    int evaluations = 0, limit = coarse ? 700 : 1800;
    while (evaluations < limit) {
        deadline();
        std::vector<double> d(poses.size(), inf);
        std::vector<int> prev(poses.size(), -1);
        d[0] = 0;
        for (size_t v = 1; v < poses.size(); v++)
            for (int e : incoming[v]) {
                auto &z = edges[e];
                double a = d[z.u] + z.weight;
                if (a < d[v]) {
                    d[v] = a;
                    prev[v] = e;
                }
            }
        if (!std::isfinite(d.back()))
            return result;
        std::vector<int> route;
        int v = int(poses.size()) - 1;
        while (v) {
            int e = prev[v];
            if (e < 0)
                return result;
            route.push_back(e);
            v = edges[e].u;
        }
        std::reverse(route.begin(), route.end());
        bool changed = false;
        for (int i : route) {
            auto &e = edges[i];
            if (e.state != 0)
                continue;
            deadline();
            e.paths = connect(poses[e.u], poses[e.v],
                              e.direct ? scales(s.truck.R, {1, 1.3, 1.7}) : scales(s.truck.R, {1}),
                              e.direct ? std::vector<double>{4, 8} : std::vector<double>{4},
                              e.direct ? .98 * pi : 2 * pi, !e.direct);
            evaluations++;
            if (e.paths.empty()) {
                e.weight = inf;
                e.state = -1;
            } else {
                e.weight = length(e.paths[0]);
                e.state = 1;
            }
            changed = true;
        }
        if (changed)
            continue;
        for (int i : route) {
            auto &e = edges[i];
            if (e.state != 1)
                continue;
            deadline();
            evaluations++;
            if (w.check(world(poses[e.u]), e.paths[e.index], 0, true))
                e.state = 2;
            else {
                e.index++;
                if (e.index >= int(e.paths.size())) {
                    e.state = -1;
                    e.weight = inf;
                } else
                    e.weight = length(e.paths[e.index]);
            }
            changed = true;
        }
        if (changed)
            continue;
        result.success = true;
        result.route = {poses[0]};
        for (int i : route) {
            auto &e = edges[i];
            result.curve = cat(std::move(result.curve), e.paths[e.index]);
            result.route.push_back(poses[e.v]);
        }
        return result;
    }
    return result;
}
static bool layered(const Scene &s, World &w, const Options &o, Result &r) {
    auto c = dag(s, w);
    if (!c.success)
        return false;
    Path p = cat(invert(c.curve), tail(s));
    if (!accept(s, w, p, r, "I-layered", true))
        return false;
    if (o.refine) {
        double previous = ctx->stopAt;
        ctx->stopAt = std::min(previous, ctx->elapsed() + o.improveBudget);
        try {
            auto f = dag(s, w, &c);
            if (f.success)
                accept(s, w, cat(invert(f.curve), tail(s)), r, "I-refined", true);
        } catch (Timeout &) {
        }
        ctx->stopAt = previous;
    }
    return true;
}
static bool one(const Scene &s, World &w, int kind, int level, Result &r, const Options &o) {
    Scope scope(kind == 2 ? "goal_side_candidates" : "start_side_candidates");
    Pose A = anchor(s), root = kind == 2 ? s.goal : s.start;
    struct Fan {
        Path p;
        Pose cusp;
    };
    std::vector<Fan> fans;
    struct Item {
        double cost;
        int fan;
        Path connection;
        bool full;
        std::uint64_t seq;
    };
    struct More {
        bool operator()(const Item &a, const Item &b) const {
            return a.cost > b.cost || (a.cost == b.cost && a.seq > b.seq);
        }
    };
    std::priority_queue<Item, std::vector<Item>, More> heap;
    std::uint64_t seq = 0;
    std::vector<double> rad = level >= 2 ? std::vector<double>{1, 1.4, 1.8, 2.4}
                                         : std::vector<double>{1, 1.4},
                        ramp = level == 3 ? std::vector<double>{1} : std::vector<double>{4, 8},
                        stems = kind == 2 ? std::vector<double>{s.tail, s.tail + 8, s.tail + 16}
                                          : std::vector<double>{0, 8};
    int da = level == 3 ? 1 : 15;
    {
        Scope generation("fan_generation");
        for (double L : stems)
            for (double R : scales(s.truck.R, rad))
                for (double rr : ramp)
                    for (int side : {-1, 1})
                        for (int angle = 0; angle <= 150; angle += da) {
                            deadline();
                            if (angle == 0 && side < 0)
                                continue;
                            Path p;
                            if (L > 0)
                                p.push_back({L, 0, 0, 1});
                            p = cat(std::move(p), turn(side * angle * pi / 180., R, rr));
                            if (p.empty() || !w.check(root, p, 0, true))
                                continue;
                            Pose q = endpoint(root, p);
                            double lower = kind == 2 ? distance(s.start, q) + 1.35 * length(p)
                                                     : length(p) + 1.35 * (distance(A, q) + s.tail);
                            int id = int(fans.size());
                            fans.push_back({std::move(p), q});
                            heap.push({lower, id, {}, false, seq++});
                        }
    }
    SelectionBudget budget;
    try {
        auto connR = scales(s.truck.R, level == 3 ? std::vector<double>{1, 1.4} : rad);
        std::vector<double> connRamp = level == 3 ? std::vector<double>{1} : std::vector<double>{4};
        while (!heap.empty()) {
            deadline();
            auto item = heap.top();
            heap.pop();
            auto &f = fans[item.fan];
            if (!item.full) {
                auto ps = connect(kind == 2 ? s.start : A, f.cusp, connR, connRamp);
                for (auto &p : ps) {
                    double cost = kind == 2 ? length(p) + 1.35 * length(f.p)
                                            : length(f.p) + 1.35 * (length(p) + s.tail);
                    heap.push({cost, item.fan, std::move(p), true, seq++});
                }
            } else {
                if (!w.check(kind == 2 ? s.start : A, item.connection, 0, true))
                    continue;
                Path p = kind == 2 ? cat(item.connection, invert(f.p))
                                   : cat(f.p, cat(invert(item.connection), tail(s)));
                if (accept(s, w, p, r, kind == 2 ? "II-fan" : "III-fan", true)) {
                    if (r.candidates >= o.candidateLimit || o.selectionSeconds <= 0)
                        return true;
                    if (r.candidates == 1)
                        budget.start(o.selectionSeconds);
                }
            }
        }
    } catch (Timeout &) {
        if (!r.success)
            throw;
    }
    return r.success;
}
static Result local(const Scene &s, World &w, const Options &o, int levels) {
    Scope scope("local_scheduler");
    Result r;
    Pose A = anchor(s);
    if (!w.check(A, tail(s), 0, true))
        return r;
    if (direct(s, w, -1, r, o))
        return r;
    if (layered(s, w, o, r))
        return r;
    int a = (!o.fixedOrder && std::abs(wrap(s.start.t - s.goal.t)) > 135 * pi / 180) ? 3 : 2,
        b = 5 - a;
    for (int level = 1; level <= levels; level++)
        for (int kind : {a, b}) {
            deadline();
            if (o.brsPolicy == 2 && kind == 3)
                continue;
            if (o.brsPolicy == 3 && kind == 2)
                continue;
            if (kind == 2 && level == 1 && direct(s, w, 1, r, o))
                return r;
            if (one(s, w, kind, level, r, o))
                return r;
        }
    return r;
}
Result brs(const Scene &s, World &w, const Options &o) {
    Result r = local(s, w, o, o.levels);
    if (r.success)
        return r;
    Scope scope("prefix_expansion");
    struct Node {
        Pose q;
        Path p;
        int depth;
    };
    std::vector<Node> queue = {{s.start, {}, 0}};
    std::set<std::array<long long, 6>> seen;
    size_t head = 0;
    int tested = 0;
    while (head < queue.size() && tested < o.maxPrefix) {
        deadline();
        Node parent = queue[head++];
        if (parent.depth >= o.depth)
            continue;
        for (double L : {4., 8., 12.})
            for (int gear : {-1, 1})
                for (double side : {-1., 1.}) {
                    deadline();
                    Path ext = prefix(gear, side, L, s.truck.R);
                    if (!w.check(parent.q, ext, 0, true))
                        continue;
                    Path pre = cat(parent.p, ext);
                    int nc = 0;
                    for (size_t i = 1; i < pre.size(); i++)
                        nc += pre[i].gear != pre[i - 1].gear;
                    if (nc > 3)
                        continue;
                    Pose q = endpoint(parent.q, ext);
                    std::array<long long, 6> key = {std::llround(q.x / 1e-6),
                                                    std::llround(q.y / 1e-6),
                                                    std::llround(wrap(q.t) / 1e-8),
                                                    gear,
                                                    nc,
                                                    parent.depth + 1};
                    if (!seen.insert(key).second)
                        continue;
                    if (tested++ >= o.maxPrefix)
                        break;
                    Scene child = s;
                    child.start = q;
                    Result sub;
                    {
                        PrefixContext retained(pre);
                        sub = local(child, w, o, 1);
                    }
                    if (sub.success) {
                        Path all = cat(pre, sub.pieces);
                        int total = 0;
                        for (size_t i = 1; i < all.size(); i++)
                            total += all[i].gear != all[i - 1].gear;
                        if (total <= 3) {
                            // accept() has already checked and timed prefix + suffix from
                            // the original start. Transfer that complete incumbent without
                            // restarting validation after the shared deadline is reached.
                            r = std::move(sub);
                            r.pieces = std::move(all);
                            r.branch = "prefix+" + r.branch;
                            r.prefixDepth = parent.depth + 1;
                            r.expanded = tested;
                            return r;
                        }
                    }
                    if (parent.depth + 1 < o.depth)
                        queue.push_back({q, std::move(pre), parent.depth + 1});
                }
    }
    r.expanded = tested;
    return r;
}
} // namespace mp
