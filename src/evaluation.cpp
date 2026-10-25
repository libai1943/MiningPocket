#include "../include/core.hpp"
namespace mp {
int World::buildTree(int a, int b) {
    deadline();
    KDNode n{inf, -inf, inf, -inf, a, b};
    for (int i = a; i < b; i++) {
        auto q = points[order[i]];
        n.xmin = std::min(n.xmin, q.x);
        n.xmax = std::max(n.xmax, q.x);
        n.ymin = std::min(n.ymin, q.y);
        n.ymax = std::max(n.ymax, q.y);
    }
    int id = int(tree.size());
    tree.push_back(n);
    if (b - a > 12) {
        int m = (a + b) / 2;
        bool x = n.xmax - n.xmin >= n.ymax - n.ymin;
        std::nth_element(order.begin() + a, order.begin() + m, order.begin() + b,
                         [&](int i, int j) {
                             return x ? points[i].x < points[j].x : points[i].y < points[j].y;
                         });
        int left = buildTree(a, m), right = buildTree(m, b);
        tree[id].left = left;
        tree[id].right = right;
    }
    return id;
}
bool World::queryTree(int id, Pose q, double c, double s, double f, double b, double h) const {
    const auto &n = tree[id];
    double dx = (n.xmin + n.xmax) / 2 - q.x, dy = (n.ymin + n.ymax) / 2 - q.y,
           ex = (n.xmax - n.xmin) / 2, ey = (n.ymax - n.ymin) / 2, x = dx * c + dy * s,
           y = -dx * s + dy * c, rx = ex * std::abs(c) + ey * std::abs(s),
           ry = ex * std::abs(s) + ey * std::abs(c);
    if (x - rx > f || x + rx < -b || y - ry > h || y + ry < -h)
        return false;
    if (x + rx <= f && x - rx >= -b && std::abs(y) + ry <= h)
        return true;
    if (n.left < 0) {
        for (int i = n.begin; i < n.end; i++) {
            auto z = points[order[i]];
            double xx = (z.x - q.x) * c + (z.y - q.y) * s, yy = -(z.x - q.x) * s + (z.y - q.y) * c;
            if (xx <= f && xx >= -b && std::abs(yy) <= h)
                return true;
        }
        return false;
    }
    return queryTree(n.left, q, c, s, f, b, h) || queryTree(n.right, q, c, s, f, b, h);
}
static double cross(Pose o, Pose a, Pose b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}
World::World(const Scene &s, const Params &p) : scene(s), params(p) {
    Scope scope("map_index");
    points = s.boundary;
    for (Pose q : s.parked)
        for (int iy = 0; iy <= int(std::floor(s.truck.width / .25)); iy++)
            for (int ix = 0; ix <= int(std::floor((s.truck.front + s.truck.rear) / .25)); ix++) {
                double x = -s.truck.rear + ix * .25, y = -s.truck.width / 2 + iy * .25;
                points.push_back({q.x + x * std::cos(q.t) - y * std::sin(q.t),
                                  q.y + x * std::sin(q.t) + y * std::cos(q.t), 0});
            }
    auto ps = s.boundary;
    std::sort(ps.begin(), ps.end(),
              [](Pose a, Pose b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    ps.erase(
        std::unique(ps.begin(), ps.end(), [](Pose a, Pose b) { return a.x == b.x && a.y == b.y; }),
        ps.end());
    for (auto q : ps) {
        while (hull.size() > 1 && cross(hull[hull.size() - 2], hull.back(), q) <= 0)
            hull.pop_back();
        hull.push_back(q);
    }
    size_t low = hull.size();
    for (int i = int(ps.size()) - 2; i >= 0; i--) {
        auto q = ps[i];
        while (hull.size() > low && cross(hull[hull.size() - 2], hull.back(), q) <= 0)
            hull.pop_back();
        hull.push_back(q);
    }
    if (hull.size() > 1)
        hull.pop_back();
    for (size_t i = 0; i < hull.size(); i++) {
        auto a = hull[i], b = hull[(i + 1) % hull.size()];
        double dx = b.x - a.x, dy = b.y - a.y, L = std::hypot(dx, dy);
        planes.push_back({dy / L, -dx / L, (dy * a.x - dx * a.y) / L});
    }
    double xmin = inf, ymin = inf, xmax = -inf, ymax = -inf;
    for (auto q : points) {
        xmin = std::min(xmin, q.x);
        ymin = std::min(ymin, q.y);
        xmax = std::max(xmax, q.x);
        ymax = std::max(ymax, q.y);
    }
    ox = xmin - 15;
    oy = ymin - 15;
    nx = int(std::ceil((xmax - ox + 15) / cell)) + 1;
    ny = int(std::ceil((ymax - oy + 15) / cell)) + 1;
    bins.resize(nx * ny);
    for (int i = 0; i < int(points.size()); i++) {
        if ((i & 1023) == 0)
            deadline();
        int x = int(std::floor((points[i].x - ox) / cell)),
            y = int(std::floor((points[i].y - oy) / cell));
        bins[x + nx * y].push_back(i);
    }
    order.resize(points.size());
    std::iota(order.begin(), order.end(), 0);
    tree.reserve(points.size() / 4);
    buildTree(0, int(points.size()));
}
bool World::collision(Pose q, double extra) {
    double f = scene.truck.front + params.margin + extra,
           b = scene.truck.rear + params.margin + extra,
           h = scene.truck.width / 2 + params.margin + extra, c = std::cos(q.t), s = std::sin(q.t),
           cx = q.x + (f - b) / 2 * c, cy = q.y + (f - b) / 2 * s;
    for (auto n : planes)
        if (cx * n[0] + cy * n[1] + (f + b) / 2 * std::abs(c * n[0] + s * n[1]) +
                h * std::abs(-s * n[0] + c * n[1]) >
            n[2] + 1e-9)
            return true;
    return queryTree(0, q, c, s, f, b, h);
}

bool World::check(Pose q, const Path &p, double step, bool continuous) {
    Scope scope("collision");
    deadline();
    if (step == 0)
        step = params.step;
    if (p.empty())
        return !collision(q);
    std::string key;
    auto add = [&](auto v) { key.append(reinterpret_cast<const char *>(&v), sizeof(v)); };
    add(q.x);
    add(q.y);
    add(q.t);
    add(step);
    add(continuous);
    for (auto &v : p) {
        add(v.L);
        add(v.k0);
        add(v.k1);
        add(v.gear);
    }
    auto found = memo.find(key);
    if (found != memo.end())
        return found->second;
    double K = 0, spacing = 0;
    for (size_t i = 0; i < p.size(); i++) {
        auto &v = p[i];
        K = std::max({K, std::abs(v.k0), std::abs(v.k1)});
        if (v.L <= 0)
            return false;
        spacing = std::max(spacing, v.L / std::ceil(v.L / step));
        if (continuous && i && std::abs(p[i - 1].k1 - v.k0) > 1e-8)
            return false;
    }
    if (K > 1 / scene.truck.R + 1e-8)
        return false;
    double rho = std::hypot(std::max(scene.truck.front, scene.truck.rear) + params.margin,
                            scene.truck.width / 2 + params.margin),
           extra = spacing / 2 * (1 + rho * K) + 1e-5;
    Pose st = q;
    for (auto &v : p) {
        int n = std::max(1, int(std::ceil(v.L / step)));
        for (int i = 0; i <= n; i += 25) {
            if ((i & 255) < 25)
                deadline();
            if (collision(advance(st, v, v.L * i / n), 0)) {
                memo[key] = false;
                return false;
            }
        }
        st = advance(st, v);
    }
    st = q;
    std::uint64_t ncheck = 0;
    for (auto &v : p) {
        int n = std::max(1, int(std::ceil(v.L / step)));
        for (int i = 0; i <= n; i++) {
            if ((ncheck++ & 63) == 0)
                deadline();
            if (collision(advance(st, v, v.L * i / n), extra)) {
                memo[key] = false;
                return false;
            }
        }
        st = advance(st, v);
    }
    memo[key] = true;
    return true;
}
bool validate(const Scene &s, World &w, const Path &p, bool continuous, double step) {
    Scope scope("final_validation");
    deadline();
    if (p.empty())
        return false;
    Pose e = endpoint(s.start, p);
    if (distance(e, s.goal) >= 1e-5 || std::abs(wrap(e.t - s.goal.t)) >= 1e-7)
        return false;
    double tail = 0;
    for (int i = int(p.size()) - 1; i >= 0; i--) {
        if (p[i].gear == -1 && std::max(std::abs(p[i].k0), std::abs(p[i].k1)) < 1e-9)
            tail += p[i].L;
        else
            break;
    }
    if (tail < s.tail - 1e-6)
        return false;
    return w.check(s.start, p, step, continuous);
}
Timing profile(const Path &p, const Truck &t, const Params &o, bool detail) {
    Scope scope("time_parameterization");
    deadline();
    Timing r;
    if (p.empty()) {
        r.valid = false;
        return r;
    }
    struct Block {
        std::vector<double> s{0}, k, cap{inf}, dp;
        int gear;
    };
    std::vector<Block> blocks;
    Block b;
    b.k = {p[0].k0};
    b.gear = p[0].gear;
    double total = 0;
    auto event = [&](double d, bool shift, double before, double after) {
        double st = std::abs(after - before) / o.omega, gt = shift ? o.shift : 0,
               duration = std::max(st, gt) + o.settle;
        r.dwell += duration;
        if (shift)
            r.cusps++;
        if (detail)
            r.events.push_back({{"distance", d},
                                {"shift", shift},
                                {"jump", std::abs(after - before) > 1e-9},
                                {"duration", duration}});
    };
    if (std::abs(std::atan(t.wb * p[0].k0)) > 1e-9)
        event(0, false, 0, std::atan(t.wb * p[0].k0));
    for (size_t j = 0; j < p.size(); j++) {
        deadline();
        const auto &v = p[j];
        if (j && (v.gear != p[j - 1].gear || std::abs(v.k0 - p[j - 1].k1) > 1e-9)) {
            blocks.push_back(std::move(b));
            event(total, v.gear != p[j - 1].gear, std::atan(t.wb * p[j - 1].k1),
                  std::atan(t.wb * v.k0));
            b = Block();
            b.k = {v.k0};
            b.gear = v.gear;
            r.stops++;
        }
        int n = std::max(2, int(std::ceil(v.L / o.ds)));
        double ds = v.L / n, s0 = b.s.back(), kp = (v.k1 - v.k0) / v.L;
        for (int i = 0; i < n; i++) {
            double ka = v.k0 + (v.k1 - v.k0) * i / n, kb = v.k0 + (v.k1 - v.k0) * (i + 1) / n,
                   kmax = std::max(std::abs(ka), std::abs(kb)),
                   kmin = ka * kb <= 0 ? 0 : std::min(std::abs(ka), std::abs(kb)),
                   rate = std::abs(t.wb * kp) / (1 + std::pow(t.wb * kmin, 2)),
                   cap =
                       std::min({v.gear > 0 ? o.vf : o.vr, std::sqrt(o.lat / std::max(kmax, 1e-30)),
                                 o.omega / std::max(rate, 1e-30)});
            b.cap.back() = std::min(b.cap.back(), cap);
            b.cap.push_back(cap);
            b.s.push_back(s0 + (i + 1) * ds);
            b.k.push_back(kb);
            b.dp.push_back(rate);
        }
        total += v.L;
    }
    blocks.push_back(std::move(b));
    if (std::abs(std::atan(t.wb * p.back().k1)) > 1e-9)
        event(total, false, std::atan(t.wb * p.back().k1), 0);
    double offset = 0;
    for (auto &z : blocks) {
        deadline();
        auto v = z.cap;
        v.front() = v.back() = 0;
        for (size_t i = 1; i < v.size(); i++)
            v[i] =
                std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2 * o.acc * (z.s[i] - z.s[i - 1])));
        for (int i = int(v.size()) - 2; i >= 0; i--)
            v[i] = std::min(v[i],
                            std::sqrt(v[i + 1] * v[i + 1] + 2 * o.brake * (z.s[i + 1] - z.s[i])));
        for (size_t i = 1; i < v.size(); i++) {
            double ds = z.s[i] - z.s[i - 1], dt = 2 * ds / (v[i] + v[i - 1]),
                   vm = std::max(v[i], v[i - 1]);
            if (!std::isfinite(dt))
                r.valid = false;
            double acc = (v[i] * v[i] - v[i - 1] * v[i - 1]) / (2 * ds);
            if (acc > o.acc + 1e-8 || -acc > o.brake + 1e-8 ||
                vm * vm * std::max(std::abs(z.k[i]), std::abs(z.k[i - 1])) > o.lat + 1e-8 ||
                vm * z.dp[i - 1] > o.omega + 1e-8)
                r.valid = false;
            r.moving += dt;
            if (detail)
                r.profile.push_back({offset + z.s[i], v[i] * z.gear, z.k[i], r.moving});
        }
        offset += z.s.back();
    }
    r.L = total;
    r.total = r.moving + r.dwell;
    r.J = r.total + o.lambda * r.stops;
    return r;
}
bool accept(const Scene &s, World &w, const Path &p, Result &r, const std::string &branch,
            bool continuous) {
    Path full = ctx && !ctx->retainedPrefix.empty() ? cat(ctx->retainedPrefix, p) : p;
    const Scene &task = ctx && !ctx->retainedPrefix.empty() ? *ctx->original : s;
    if (!validate(task, w, full, continuous))
        return false;
    Timing t = profile(full, s.truck, w.params);
    if (!t.valid)
        return false;
    r.candidates++;
    double now = ctx ? ctx->elapsed() : 0;
    if (r.first < 0)
        r.first = now;
    if (!r.success || t.J < r.timing.J - 1e-9) {
        r.success = true;
        r.pieces = p;
        r.timing = t;
        r.branch = branch;
        r.reason = "success";
    }
    return true;
}

Grid::Grid(World &w, Pose goal) {
    Scope scope("grid_heuristic");
    double xmax = -inf, ymax = -inf;
    ox = oy = inf;
    for (auto q : w.points) {
        ox = std::min(ox, q.x);
        oy = std::min(oy, q.y);
        xmax = std::max(xmax, q.x);
        ymax = std::max(ymax, q.y);
    }
    ox = std::floor(ox) - 1;
    oy = std::floor(oy) - 1;
    nx = int(std::ceil(xmax) + 1 - ox) + 1;
    ny = int(std::ceil(ymax) + 1 - oy) + 1;
    std::vector<bool> occ(nx * ny, false);
    for (auto q : w.points) {
        int x = int(std::round(q.x - ox)), y = int(std::round(q.y - oy));
        occ[x + nx * y] = true;
    }
    for (int y = 0; y < ny; y++) {
        deadline();
        for (int x = 0; x < nx; x++) {
            for (auto n : w.planes)
                if ((ox + x) * n[0] + (oy + y) * n[1] > n[2] + 1e-9) {
                    occ[x + nx * y] = true;
                    break;
                }
        }
    }
    int tx = std::clamp(int(std::round(goal.x - ox)), 0, nx - 1),
        ty = std::clamp(int(std::round(goal.y - oy)), 0, ny - 1);
    d.assign(nx * ny, inf);
    int id = tx + nx * ty;
    d[id] = 0;
    occ[id] = false;
    using Q = std::pair<double, int>;
    std::priority_queue<Q, std::vector<Q>, std::greater<Q>> heap;
    heap.push({0, id});
    int count = 0;
    while (!heap.empty()) {
        if ((count++ & 255) == 0)
            deadline();
        auto [v, i] = heap.top();
        heap.pop();
        if (v > d[i])
            continue;
        int x = i % nx, y = i / nx;
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                if (!dx && !dy)
                    continue;
                int a = x + dx, b = y + dy;
                if (a < 0 || b < 0 || a >= nx || b >= ny)
                    continue;
                int j = a + nx * b;
                if (occ[j])
                    continue;
                double nv = v + std::hypot(double(dx), double(dy));
                if (nv < d[j]) {
                    d[j] = nv;
                    heap.push({nv, j});
                }
            }
    }
}
double Grid::at(Pose q, Pose goal) const {
    int x = int(std::round(q.x - ox)), y = int(std::round(q.y - oy));
    if (x < 0 || y < 0 || x >= nx || y >= ny)
        return distance(q, goal);
    double v = d[x + nx * y];
    return std::isfinite(v) ? v : distance(q, goal);
}
} // namespace mp
