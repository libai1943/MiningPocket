#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <filesystem>
#include <random>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include "../third_party/json.hpp"
namespace mp {
using json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using C = std::complex<double>;
constexpr double pi = 3.1415926535897932384626433832795,
                 inf = std::numeric_limits<double>::infinity();
inline double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}
inline double mod2pi(double a) {
    double b = std::fmod(a, 2 * pi);
    return b < 0 ? b + 2 * pi : b;
}
struct Pose {
    double x = 0, y = 0, t = 0;
};
inline double distance(Pose a, Pose b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}
inline Pose pose(const json &j) {
    return {j[0], j[1], j[2]};
}
inline json jpose(Pose a) {
    return {a.x, a.y, a.t};
}
struct Piece {
    double L = 0, k0 = 0, k1 = 0;
    int gear = 1;
};
using Path = std::vector<Piece>;
inline double length(const Path &p) {
    double s = 0;
    for (auto &v : p)
        s += v.L;
    return s;
}
inline Path cat(Path a, const Path &b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}
inline Path invert(Path p) {
    std::reverse(p.begin(), p.end());
    for (auto &v : p) {
        std::swap(v.k0, v.k1);
        v.gear = -v.gear;
    }
    return p;
}
inline json jpath(const Path &p) {
    json j = json::array();
    for (auto &v : p)
        j.push_back({v.L, v.k0, v.k1, v.gear});
    return j;
}
inline Path path(const json &j) {
    Path p;
    for (auto &v : j)
        p.push_back({v[0], v[1], v[2], v[3]});
    return p;
}
struct Truck {
    double front = 8.32, rear = 2.08, width = 4.9, wb = 6.22, R = 15;
};
struct Scene {
    int id;
    Pose start, goal;
    Truck truck;
    double tail;
    std::vector<Pose> boundary, parked;
};
struct Params {
    double margin = .5, step = .04, verifyStep = .02, ds = .1, vf = 4, vr = 2, acc = .5, brake = .7,
           lat = 1, omega = 5 * pi / 180, shift = 2, settle = .5, lambda = 10, deadline = 10,
           reserve = .05;
};
struct Timeout {};
struct Stat {
    double seconds = 0;
    std::uint64_t calls = 0;
};
struct Context {
    const Scene *original = nullptr;
    Path retainedPrefix;
    Clock::time_point begun;
    double budget, stopAt;
    std::map<std::string, Stat> stats;
    std::vector<std::string> stack;
    Clock::time_point last;
    std::uint64_t checks = 0;
    Context(double b, double r)
        : begun(Clock::now()), budget(b), stopAt(std::max(0., b - r)), last(begun) {}
    double elapsed() const {
        return std::chrono::duration<double>(Clock::now() - begun).count();
    }
    void check() {
        ++checks;
        if (elapsed() >= stopAt)
            throw Timeout();
    }
    void charge() {
        auto n = Clock::now();
        if (!stack.empty())
            stats[stack.back()].seconds += std::chrono::duration<double>(n - last).count();
        last = n;
    }
    void enter(const std::string &s) {
        charge();
        stack.push_back(s);
        stats[s].calls++;
    }
    void leave() {
        charge();
        stack.pop_back();
    }
};
extern thread_local Context *ctx;
inline void deadline() {
    if (ctx)
        ctx->check();
}
struct Scope {
    bool active;
    Scope(const std::string &s) : active(ctx != nullptr) {
        if (active)
            ctx->enter(s);
    }
    ~Scope() {
        if (active)
            ctx->leave();
    }
};
struct Timing {
    bool valid = true;
    double moving = 0, dwell = 0, total = 0, J = 0, L = 0;
    int stops = 0, cusps = 0;
    json events = json::array(), profile = json::array();
};
struct Result {
    bool success = false;
    Path pieces;
    Timing timing;
    std::string reason = "exhausted", branch;
    double seconds = 0, first = -1;
    int expanded = 0, candidates = 0, prefixDepth = 0;
    json diagnostics;
};
struct Options {
    bool first = true, refine = false, diagnosticFirstOnly = false, fixedOrder = false;
    int candidateLimit = 1;
    double selectionSeconds = 0;
    double improveBudget = .2;
    int brsPolicy = 0, maxNodes = 150000, levels = 3, maxPrefix = 128, depth = 2;
};
Scene readScene(const std::string &file);
Params readParams(const std::string &file);
C integrate(double th, double k, double sigma, double L);
Pose advance(Pose q, const Piece &p, double L = -1);
Pose endpoint(Pose q, const Path &p);
Path turn(double phi, double R, double ramp);
C displacement(double phi, double R, double ramp);
std::vector<Path> connect(Pose a, Pose b, const std::vector<double> &radii,
                          const std::vector<double> &ramps, double maxTurn = 1.8 * pi,
                          bool local = false);
Path prefix(int gear, double side, double L, double R, double ramp = 1);
struct KDNode {
    double xmin, xmax, ymin, ymax;
    int begin, end, left = -1, right = -1;
};
struct World {
    std::vector<KDNode> tree;
    std::vector<int> order;
    int buildTree(int a, int b);
    bool queryTree(int node, Pose q, double c, double s, double f, double b, double h) const;
    Scene scene;
    Params params;
    double cell = 8, ox, oy;
    int nx, ny;
    std::vector<Pose> points, hull;
    std::vector<std::vector<int>> bins;
    std::vector<std::array<double, 3>> planes;
    std::unordered_map<std::string, bool> memo;
    World(const Scene &s, const Params &p);
    bool collision(Pose q, double extra = 0);
    bool check(Pose q, const Path &p, double step = 0, bool continuous = false);
};
Timing profile(const Path &p, const Truck &t, const Params &o, bool detail = false);
bool validate(const Scene &s, World &w, const Path &p, bool continuous = false, double step = 0);
bool accept(const Scene &s, World &w, const Path &p, Result &r, const std::string &branch,
            bool continuous = false);
Result brs(const Scene &s, World &w, const Options &o);
Result search(const Scene &s, World &w, const std::string &method, const Options &o);
Result biagt(const Scene &s, World &w, const Options &o);
Result ccsearch(const Scene &s, World &w, const Options &o);
std::vector<Path> rsWords(Pose a, Pose b, double R);
std::vector<Path> dubinsWords(Pose a, Pose b, double R);
double geoCost(const Path &p, int lastGear = 0, double lastK = 0);
double rsDistance(Pose a, Pose b, double R);
struct Grid {
    double ox, oy, res = 1;
    int nx, ny;
    std::vector<double> d;
    Grid(World &w, Pose goal);
    double at(Pose q, Pose goal) const;
};
json runQuery(const Scene &s, const Params &p, const std::string &method, const Options &o);
} // namespace mp
