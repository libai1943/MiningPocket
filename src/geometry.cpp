#include "../include/core.hpp"
namespace mp {
thread_local Context *ctx = nullptr;
Scene readScene(const std::string &file) {
    std::ifstream f(file);
    json j;
    f >> j;
    Scene s;
    s.id = j["id"].is_number_integer() ? j["id"].get<int>()
                                       : std::stoi(j["id"].get<std::string>().substr(1));
    s.start = pose(j["start"]);
    s.goal = pose(j["goal"]);
    s.tail = j["minimumStraight"];
    auto &t = j["truck"];
    s.truck = {t["front"], t["rear"], t["width"], t["wheelbase"], t["radius"]};
    for (auto &v : j["boundary"])
        s.boundary.push_back({v[0], v[1], 0});
    auto &pk = j["parkedTruck"];
    if (!pk.empty()) {
        if (pk[0].is_array()) {
            for (auto &q : pk)
                s.parked.push_back(pose(q));
        } else
            s.parked.push_back(pose(pk));
    }
    return s;
}
Params readParams(const std::string &file) {
    std::ifstream f(file);
    json j;
    f >> j;
    Params p;
    p.margin = j["margin"];
    p.step = j["collision_step"];
    p.verifyStep = j["verification_step"];
    p.ds = j["timing_step"];
    p.vf = j["v_forward"];
    p.vr = j["v_reverse"];
    p.acc = j["acceleration"];
    p.brake = j["braking"];
    p.lat = j["lateral_acceleration"];
    p.omega = j["steering_rate"];
    p.shift = j["gear_time"];
    p.settle = j["settle_time"];
    p.lambda = j["stop_penalty"];
    p.deadline = j["deadline_seconds"];
    p.reserve = j["return_reserve_seconds"];
    return p;
}
static const double gx[4] = {.18343464249564980494, .52553240991632898582, .79666647741362673959,
                             .96028985649753623168};
static const double gw[4] = {.36268378337836198297, .31370664587788728734, .22238103445337447054,
                             .10122853629037625915};
C integrate(double th, double k, double sigma, double L) {
    if (sigma == 0) {
        if (k == 0)
            return std::polar(L, th);
        return (2 * std::sin(.5 * k * L) / k) * std::polar(1., th + .5 * k * L);
    }
    // For a zero-initial-curvature ramp, integrate its convergent Fresnel series.
    // |alpha| <= 1: the omitted tail is bounded by L*exp(1)/15! < 2.1e-12*L.
    // Native turn ramps use |alpha| <= 0.267, giving a much smaller truncation error.
    const double alpha = .5 * sigma * L * L;
    if (k == 0 && std::abs(alpha) <= 1) {
        const double z = alpha * alpha;
        const double realPart =
            1 +
            z * (-1. / 10 +
                 z * (1. / 216 + z * (-1. / 9360 +
                                      z * (1. / 685440 + z * (-1. / 76204800 +
                                                              z * (1. / 11975040000 +
                                                                   z * (-1. / 2528170444800)))))));
        const double imagPart =
            alpha *
            (1. / 3 +
             z * (-1. / 42 +
                  z * (1. / 1320 +
                       z * (-1. / 75600 + z * (1. / 6894720 +
                                               z * (-1. / 918086400 + z * (1. / 168129561600)))))));
        return std::polar(L, th) * C(realPart, imagPart);
    }
    C z = 0;
    for (int i = 0; i < 4; i++)
        for (int sg : {-1, 1}) {
            double x = L * (1 + sg * gx[i]) / 2;
            z += gw[i] * C(std::cos(th + k * x + .5 * sigma * x * x),
                           std::sin(th + k * x + .5 * sigma * x * x));
        }
    return z * (L / 2);
}
Pose advance(Pose q, const Piece &p, double L) {
    if (L < 0)
        L = p.L;
    if (p.L <= 0)
        return q;
    double k = p.gear * p.k0, sg = p.gear * (p.k1 - p.k0) / p.L;
    C z = double(p.gear) * integrate(q.t, k, sg, L);
    return {q.x + z.real(), q.y + z.imag(), q.t + k * L + .5 * sg * L * L};
}
Pose endpoint(Pose q, const Path &p) {
    for (auto &v : p)
        q = advance(q, v);
    return q;
}
Path turn(double phi, double R, double ramp) {
    double a = std::abs(phi);
    if (a <= 64 * std::numeric_limits<double>::epsilon())
        return {};
    double sg = phi < 0 ? -1 : 1, sigma = 1 / (R * ramp) * std::min(1., std::sqrt(a / 1e-6)),
           peak = std::min(1 / R, std::sqrt(a * sigma)), lr = peak / sigma,
           lc = std::max(0., (a - peak * lr) / peak);
    Path p;
    if (lr > 1e-9)
        p.push_back({lr, 0, sg * peak, 1});
    if (lc > 1e-9)
        p.push_back({lc, sg * peak, sg * peak, 1});
    if (lr > 1e-9)
        p.push_back({lr, sg * peak, 0, 1});
    return p;
}
C displacement(double phi, double R, double ramp) {
    double a = std::abs(phi);
    if (a <= 64 * std::numeric_limits<double>::epsilon())
        return 0;
    double sg = phi < 0 ? -1 : 1, sigma = 1 / (R * ramp) * std::min(1., std::sqrt(a / 1e-6)),
           peak = std::min(1 / R, std::sqrt(a * sigma)), lr = peak / sigma,
           lc = std::max(0., (a - peak * lr) / peak);
    C z = integrate(0, 0, sg * sigma, lr);
    return z + integrate(.5 * sg * sigma * lr * lr, sg * peak, 0, lc) +
           std::polar(1., phi) * std::conj(z);
}
static double root(const std::function<double(double)> &f, double a, double b) {
    double fa = f(a);
    for (int n = 0; n < 55; n++) {
        if ((n & 7) == 0)
            deadline();
        double c = (a + b) * .5, fc = f(c);
        if (std::abs(fc) < 1e-12 || b - a < 1e-12)
            return c;
        if (fa * fc <= 0) {
            b = c;
        } else {
            a = c;
            fa = fc;
        }
    }
    return (a + b) * .5;
}
std::vector<Path> connect(Pose a, Pose b, const std::vector<double> &radii,
                          const std::vector<double> &ramps, double maxTurn, bool local) {
    Scope scope("connector");
    std::vector<Path> out;
    C delta(b.x - a.x, b.y - a.y);
    for (double R : radii)
        for (double ramp : ramps)
            for (int s1 : {-1, 1})
                for (int s2 : {-1, 1}) {
                    deadline();
                    if (local && (s1 != 1 || s2 != 1))
                        continue;
                    auto residual = [&](double beta) {
                        double p1 = local ? beta - a.t : s1 * mod2pi(s1 * (beta - a.t)),
                               p2 = local ? b.t - beta : s2 * mod2pi(s2 * (b.t - beta));
                        if (!local) {
                            if (std::abs(p1 - s1 * 2 * pi) < 1e-9)
                                p1 = 0;
                            if (std::abs(p2 - s2 * 2 * pi) < 1e-9)
                                p2 = 0;
                        }
                        C v = (delta - std::polar(1., a.t) * displacement(p1, R, ramp) -
                               std::polar(1., beta) * displacement(p2, R, ramp)) *
                              std::polar(1., -beta);
                        return std::array<double, 4>{v.imag(), v.real(), p1, p2};
                    };
                    std::vector<double> roots, xx, special = {a.t, b.t, std::arg(delta)};
                    for (double v : special) {
                        auto f = residual(v);
                        if (std::abs(f[0]) < 1e-9 && std::abs(f[2]) <= maxTurn &&
                            std::abs(f[3]) <= maxTurn)
                            roots.push_back(v);
                    }
                    if (local) {
                        for (int i = 0; i <= 40; i++)
                            xx.push_back(-pi / 2 + .02 + i * (pi - .04) / 40);
                        xx.insert(xx.end(), special.begin(), special.end());
                        std::sort(xx.begin(), xx.end());
                        xx.erase(std::unique(xx.begin(), xx.end()), xx.end());
                    } else {
                        std::vector<double> cuts = {-pi, pi, wrap(a.t), wrap(b.t)};
                        std::sort(cuts.begin(), cuts.end());
                        cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
                        for (size_t j = 1; j < cuts.size(); j++) {
                            int n = std::max(
                                3, int(std::ceil((cuts[j] - cuts[j - 1]) / (pi / 72))) + 1);
                            for (int k = 0; k < n; k++)
                                xx.push_back(cuts[j - 1] + 1e-8 +
                                             (cuts[j] - cuts[j - 1] - 2e-8) * k / (n - 1));
                        }
                    }
                    if (!xx.empty()) {
                        auto f0 = residual(xx[0]);
                        for (size_t i = 1; i < xx.size(); i++) {
                            if ((i & 15) == 0)
                                deadline();
                            auto f1 = residual(xx[i]);
                            if (std::abs(f0[2]) <= maxTurn && std::abs(f0[3]) <= maxTurn &&
                                std::abs(f1[2]) <= maxTurn && std::abs(f1[3]) <= maxTurn) {
                                if (f0[0] * f1[0] < 0)
                                    roots.push_back(root([&](double v) { return residual(v)[0]; },
                                                         xx[i - 1], xx[i]));
                                else if (std::abs(f0[0]) < 1e-9)
                                    roots.push_back(xx[i - 1]);
                            }
                            f0 = f1;
                        }
                    }
                    std::vector<double> unique;
                    for (double beta : roots) {
                        bool dup = false;
                        for (double v : unique)
                            if (std::abs(v - beta) <= 1e-10) {
                                dup = true;
                                break;
                            }
                        if (dup)
                            continue;
                        unique.push_back(beta);
                        auto f = residual(beta);
                        if (f[1] < -1e-7 || std::abs(f[0]) > 1e-6 || std::abs(f[2]) > maxTurn ||
                            std::abs(f[3]) > maxTurn)
                            continue;
                        Path p = turn(f[2], R, ramp);
                        if (f[1] > 1e-8)
                            p.push_back({f[1], 0, 0, 1});
                        p = cat(p, turn(f[3], R, ramp));
                        if (p.empty())
                            continue;
                        Pose q = endpoint(a, p);
                        if (distance(q, b) < 1e-5 && std::abs(wrap(q.t - b.t)) < 1e-7)
                            out.push_back(std::move(p));
                    }
                }
    std::stable_sort(out.begin(), out.end(),
                     [](const Path &a, const Path &b) { return length(a) < length(b); });
    return out;
}
Path prefix(int gear, double side, double L, double R, double ramp) {
    double r = std::min(ramp, L / 2), k = side / R;
    Path p = {{r, 0, k, gear}};
    if (L > 2 * r)
        p.push_back({L - 2 * r, k, k, gear});
    p.push_back({r, k, 0, gear});
    return p;
}
double geoCost(const Path &p, int lastGear, double lastK) {
    double v = 0;
    for (auto &x : p) {
        v += x.L * (x.gear < 0 ? 1.35 : 1);
        if (lastGear)
            v += 5 * (x.gear != lastGear);
        v += 2 * std::abs(x.k0 - lastK) * 15;
        lastGear = x.gear;
        lastK = x.k1;
    }
    return v;
}
} // namespace mp
