#include "../include/core.hpp"
namespace mp {
struct Node {
    Pose q;
    int parent = -1;
    Path move;
    double g = 0, h = 0;
    int gear = 0, cusps = 0;
    std::array<double, 2> priority{0, 0};
    std::array<bool, 2> untried{true, true};
};
struct Entry {
    double f;
    int id;
    std::uint64_t seq;
};
struct Greater {
    bool operator()(const Entry &a, const Entry &b) const {
        return a.f > b.f || (a.f == b.f && a.seq > b.seq);
    }
};
using Heap = std::priority_queue<Entry, std::vector<Entry>, Greater>;
static std::string key(Pose q, int gear, int cusps = -1) {
    int ang = int(std::llround(q.t / (5 * pi / 180))) % 72;
    if (ang < 0)
        ang += 72;
    return std::to_string(std::llround(q.x)) + "," + std::to_string(std::llround(q.y)) + "," +
           std::to_string(ang) + "," + std::to_string(gear) + "," + std::to_string(cusps);
}
static Path trace(const std::vector<Node> &n, int id) {
    std::vector<int> ids;
    while (n[id].parent >= 0) {
        ids.push_back(id);
        id = n[id].parent;
    }
    Path p;
    for (auto it = ids.rbegin(); it != ids.rend(); it++)
        p = cat(std::move(p), n[*it].move);
    return p;
}
static Pose anchor(const Scene &s) {
    return advance(s.goal, {s.tail, 0, 0, 1});
}
static bool shot(Pose a, Pose b, World &w, Path &answer, int lastGear = 0, double lastK = 0) {
    Scope scope("analytic_shot");
    auto words = rsWords(a, b, w.scene.truck.R);
    std::stable_sort(words.begin(), words.end(), [&](const Path &p, const Path &q) {
        return geoCost(p, lastGear, lastK) < geoCost(q, lastGear, lastK);
    });
    for (auto &p : words) {
        deadline();
        if (w.check(a, p)) {
            answer = p;
            return true;
        }
    }
    return false;
}
Result search(const Scene &s, World &w, const std::string &method, const Options &o) {
    Result r;
    try {
        Scope scope("search_expansion");
        bool reverse = method == "GHA";
        Pose A = anchor(s), start = reverse ? A : s.start, goal = reverse ? s.start : A;
        Path tail = {{s.tail, 0, 0, -1}};
        if (!w.check(A, tail))
            return r;
        std::unique_ptr<Grid> grid;
        if (!reverse)
            grid = std::make_unique<Grid>(w, goal);
        std::vector<Node> nodes(1);
        nodes[0].q = start;
        nodes[0].h =
            reverse ? 0 : std::max(rsDistance(start, goal, s.truck.R), grid->at(start, goal));
        Heap heap;
        std::uint64_t seq = 0;
        heap.push({nodes[0].h, 0, seq++});
        std::unordered_map<std::string, double> seen;
        seen[key(start, 0)] = 0;
        while (!heap.empty() && nodes.size() < size_t(o.maxNodes)) {
            deadline();
            auto entry = heap.top();
            heap.pop();
            int id = entry.id;
            Node n = nodes[id];
            auto ky = key(n.q, n.gear);
            if (seen[ky] < n.g - 1e-8)
                continue;
            r.expanded++;
            if (r.expanded == 1 || r.expanded % 10 == 0) {
                std::vector<Path> pool;
                if (reverse) {
                    auto all = dubinsWords(n.q, goal, s.truck.R);
                    for (int gear : {1, -1}) {
                        int best = -1;
                        double L = inf;
                        for (int j = 0; j < int(all.size()); j++)
                            if (!all[j].empty() && all[j][0].gear == gear && length(all[j]) < L) {
                                L = length(all[j]);
                                best = j;
                            }
                        if (best >= 0 && w.check(n.q, all[best]))
                            pool.push_back(all[best]);
                    }
                } else {
                    Path p;
                    if (shot(n.q, goal, w, p, n.gear, n.move.empty() ? 0 : n.move.back().k1))
                        pool.push_back(p);
                }
                for (auto &p : pool) {
                    Path q = cat(trace(nodes, id), p);
                    if (reverse)
                        q = invert(q);
                    q = cat(std::move(q), tail);
                    if (accept(s, w, q, r, method) &&
                        (!reverse || o.diagnosticFirstOnly || r.candidates >= 100))
                        return r;
                }
            }
            for (int gear : {1, -1})
                for (double steer : {-1., -.5, 0., .5, 1.}) {
                    deadline();
                    Path move = {{3, steer / s.truck.R, steer / s.truck.R, gear}};
                    Pose q = endpoint(n.q, move);
                    Path phys = move;
                    if (reverse)
                        phys[0].gear = -gear;
                    double g = n.g + geoCost(phys, reverse ? -n.gear : n.gear,
                                             n.move.empty() ? 0 : n.move.back().k1);
                    std::string k = key(q, gear);
                    auto it = seen.find(k);
                    if (it != seen.end() && it->second <= g + 1e-8)
                        continue;
                    if (!w.check(n.q, move))
                        continue;
                    double h =
                        reverse ? 0 : std::max(rsDistance(q, goal, s.truck.R), grid->at(q, goal));
                    if (nodes.size() >= size_t(o.maxNodes))
                        break;
                    int idx = int(nodes.size());
                    nodes.push_back({q, id, move, g, h, gear, 0});
                    seen[k] = g;
                    heap.push({g + h, idx, seq++});
                }
        }
    } catch (Timeout &) {
        r.reason = r.success ? "budget_with_incumbent" : "timeout";
    }
    return r;
}
static std::vector<Path> ccwords(Pose start, Pose goal, World &w) {
    Scope scope("cc_word_shot");
    std::vector<Path> solutions;
    auto words = rsWords(start, goal, w.scene.truck.R);
    for (auto &word : words) {
        deadline();
        Pose q = start;
        std::vector<Pose> poses = {q};
        std::vector<int> gears;
        for (size_t k = 0; k < word.size(); k++) {
            q = advance(q, word[k]);
            if (k + 1 == word.size() || word[k + 1].gear != word[k].gear) {
                poses.push_back(q);
                gears.push_back(word[k].gear);
            }
        }
        if (gears.size() > 4)
            continue;
        Path all;
        bool valid = true;
        for (size_t k = 0; k < gears.size(); k++) {
            int gear = gears[k];
            Pose a = poses[k], b = poses[k + 1];
            auto pp = connect(gear < 0 ? b : a, gear < 0 ? a : b,
                              {w.scene.truck.R, 1.4 * w.scene.truck.R}, {1, 4});
            bool found = false;
            for (auto &p : pp) {
                Path exec = gear < 0 ? invert(p) : p;
                if (w.check(a, exec, 0, true)) {
                    all = cat(std::move(all), exec);
                    found = true;
                    break;
                }
            }
            if (!found) {
                valid = false;
                break;
            }
        }
        if (valid) {
            solutions.push_back(std::move(all));
            if (solutions.size() >= 3)
                break;
        }
    }
    return solutions;
}
Result ccsearch(const Scene &s, World &w, const Options &o) {
    Result r;
    try {
        Scope scope("cc_search_expansion");
        Pose A = anchor(s);
        Path tail = {{s.tail, 0, 0, -1}};
        if (!w.check(A, tail, 0, true))
            return r;
        Grid grid(w, A);
        std::vector<Node> nodes(1);
        nodes[0].q = s.start;
        Heap heap;
        std::uint64_t seq = 0;
        heap.push({distance(s.start, A), 0, seq++});
        std::unordered_map<std::string, double> seen;
        seen[key(s.start, 0, 0)] = 0;
        while (!heap.empty() && nodes.size() < size_t(o.maxNodes)) {
            deadline();
            int id = heap.top().id;
            heap.pop();
            Node n = nodes[id];
            if (seen[key(n.q, n.gear, n.cusps)] < n.g - 1e-8)
                continue;
            r.expanded++;
            if (r.expanded == 1 || r.expanded % 10 == 0) {
                Path pre = trace(nodes, id);
                for (int gear : {-1, 1}) {
                    auto ps = connect(gear < 0 ? A : n.q, gear < 0 ? n.q : A,
                                      {s.truck.R, 1.3 * s.truck.R, 1.7 * s.truck.R}, {4, 8});
                    int accepted = 0;
                    for (auto &p : ps) {
                        deadline();
                        Path exec = gear < 0 ? invert(p) : p;
                        if (!w.check(n.q, exec, 0, true))
                            continue;
                        Path all = cat(cat(pre, exec), tail);
                        int nc = 0;
                        for (size_t i = 1; i < all.size(); i++)
                            nc += all[i].gear != all[i - 1].gear;
                        if (nc > 3)
                            continue;
                        if (accept(s, w, all, r, "CC-Astar", true)) {
                            if (o.diagnosticFirstOnly)
                                return r;
                            if (++accepted >= 3)
                                break;
                        }
                    }
                }
                if (!r.success) {
                    auto ps = ccwords(n.q, A, w);
                    for (auto &p : ps) {
                        Path all = cat(cat(pre, p), tail);
                        int nc = 0;
                        for (size_t i = 1; i < all.size(); i++)
                            nc += all[i].gear != all[i - 1].gear;
                        if (nc <= 3 && accept(s, w, all, r, "CC-Astar-words", true) &&
                            o.diagnosticFirstOnly)
                            return r;
                    }
                }
                if (r.success)
                    return r;
            }
            for (double L : {4., 8., 12.})
                for (int gear : {1, -1}) {
                    int nc = n.cusps + (n.gear && n.gear != gear);
                    if (nc > 3)
                        continue;
                    for (double side : {-1., -.5, 0., .5, 1.}) {
                        deadline();
                        Path p = side == 0 ? Path{{L, 0, 0, gear}}
                                           : prefix(gear, side < 0 ? -1 : 1, L,
                                                    s.truck.R / std::abs(side));
                        Pose q = endpoint(n.q, p);
                        double g = n.g + L * (gear < 0 ? 1.35 : 1) + 5 * (nc - n.cusps);
                        std::string ky = key(q, gear, nc);
                        auto it = seen.find(ky);
                        if (it != seen.end() && it->second <= g + 1e-8)
                            continue;
                        if (!w.check(n.q, p, 0, true))
                            continue;
                        double h = std::max(
                            {distance(q, A), s.truck.R * std::abs(wrap(q.t - A.t)), grid.at(q, A)});
                        int idx = int(nodes.size());
                        nodes.push_back({q, id, p, g, h, gear, nc});
                        seen[ky] = g;
                        heap.push({g + h, idx, seq++});
                    }
                }
        }
    } catch (Timeout &) {
        r.reason = r.success ? "budget_with_incumbent" : "timeout";
    }
    return r;
}
static double metric(Pose a, Pose b, double R) {
    return std::hypot(distance(a, b), R * wrap(a.t - b.t));
}
Result biagt(const Scene &s, World &w, const Options &o) {
    Result r;
    try {
        Scope scope("bidirectional_search");
        Pose A = anchor(s);
        Path tail = {{s.tail, 0, 0, -1}};
        if (!w.check(A, tail))
            return r;
        struct Tree {
            std::vector<Node> nodes;
            Heap heap;
            Pose target;
        };
        std::array<Tree, 2> trees;
        std::uint64_t seq = 0;
        for (int i = 0; i < 2; i++) {
            Pose q = i ? A : s.start, target = i ? s.start : A;
            Node n;
            n.q = q;
            n.h = rsDistance(q, target, s.truck.R);
            trees[i].nodes.push_back(n);
            trees[i].heap.push({1.5 * n.h, 0, seq++});
            trees[i].target = target;
        }
        int active = 1;
        bool close = false;
        while (r.expanded < o.maxNodes) {
            deadline();
            if (trees[0].heap.empty() && trees[1].heap.empty())
                break;
            if (trees[0].heap.empty())
                active = 1;
            else if (trees[1].heap.empty())
                active = 0;
            else if (!close)
                active = 1 - active;
            else
                active = trees[1].nodes[trees[1].heap.top().id].h <
                                 trees[0].nodes[trees[0].heap.top().id].h
                             ? 1
                             : 0;
            Tree &ta = trees[active];
            Tree &tb = trees[1 - active];
            int id = ta.heap.top().id;
            ta.heap.pop();
            Node n = ta.nodes[id];
            r.expanded++;
            if (n.untried[0] && n.untried[1]) {
                std::vector<std::pair<double, int>> ds;
                for (int j = 0; j < int(tb.nodes.size()); j++)
                    ds.push_back({metric(n.q, tb.nodes[j].q, s.truck.R), j});
                std::stable_sort(ds.begin(), ds.end());
                if (ds.front().first <= 20)
                    close = true;
                if (r.expanded <= 2 || r.expanded % 10 == 0 || ds.front().first < 2) {
                    std::vector<int> targets;
                    for (int j = 0; j < std::min(3, int(ds.size())); j++)
                        if (ds[j].first <= 20)
                            targets.push_back(ds[j].second);
                    if (metric(n.q, tb.nodes[0].q, s.truck.R) <= 20) {
                        targets.erase(std::remove(targets.begin(), targets.end(), 0),
                                      targets.end());
                        targets.insert(targets.begin(), 0);
                    }
                    for (int j : targets) {
                        deadline();
                        Pose aa = active == 0 ? n.q : tb.nodes[j].q,
                             bb = active == 0 ? tb.nodes[j].q : n.q;
                        Path conn;
                        if (shot(aa, bb, w, conn)) {
                            Path start = active == 0 ? trace(ta.nodes, id) : trace(tb.nodes, j),
                                 end = active == 0 ? trace(tb.nodes, j) : trace(ta.nodes, id);
                            Path all = cat(cat(cat(start, conn), invert(end)), tail);
                            if (accept(s, w, all, r, "BIAGT"))
                                return r;
                        }
                    }
                }
            }
            if (n.untried[0] && n.untried[1] && n.parent >= 0)
                n.priority = ta.nodes[n.parent].priority;
            int mode = n.untried[0] && (!n.untried[1] || n.priority[0] <= n.priority[1]) ? 0 : 1,
                gear = mode ? -1 : 1;
            n.untried[mode] = false;
            double bestF = inf;
            for (double kk : {-1., -.5, 0., .5, 1.}) {
                deadline();
                Path p = {{3, kk / s.truck.R, kk / s.truck.R, gear}};
                Pose q = endpoint(n.q, p);
                double near = inf;
                for (auto &x : ta.nodes)
                    near = std::min(near, metric(q, x.q, s.truck.R));
                if (near < 1 || !w.check(n.q, p))
                    continue;
                Path physical = p;
                if (active)
                    physical[0].gear = -gear;
                double g = n.g + geoCost(physical, active ? -n.gear : n.gear,
                                         n.move.empty() ? 0 : n.move.back().k1);
                std::vector<std::pair<double, int>> nearby;
                double nearest = inf;
                for (int j = 0; j < int(tb.nodes.size()); j++) {
                    double d = metric(q, tb.nodes[j].q, s.truck.R);
                    nearest = std::min(nearest, d);
                    if (d < 30)
                        nearby.push_back({distance(q, tb.nodes[j].q) + tb.nodes[j].g, j});
                }
                double h = inf;
                if (nearby.empty())
                    h = rsDistance(q, ta.target, s.truck.R);
                else {
                    std::stable_sort(nearby.begin(), nearby.end());
                    for (auto [lb, j] : nearby) {
                        if (lb >= h)
                            break;
                        deadline();
                        h = std::min(h, rsDistance(q, tb.nodes[j].q, s.truck.R) + tb.nodes[j].g);
                    }
                }
                if (nearest <= 20)
                    close = true;
                Node child;
                child.q = q;
                child.parent = id;
                child.move = p;
                child.g = g;
                child.h = h;
                child.gear = gear;
                child.priority = n.priority;
                int idx = int(ta.nodes.size());
                ta.nodes.push_back(child);
                double f = g + 1.5 * h;
                ta.heap.push({f, idx, seq++});
                bestF = std::min(bestF, f);
            }
            if (std::isfinite(bestF))
                n.priority[mode] = bestF;
            ta.nodes[id].priority = n.priority;
            ta.nodes[id].untried = n.untried;
            if (n.untried[0] || n.untried[1])
                ta.heap.push({n.g + 1.5 * n.h, id, seq++});
        }
    } catch (Timeout &) {
        r.reason = r.success ? "budget_with_incumbent" : "timeout";
    }
    return r;
}
} // namespace mp
