/// @file amp_bench.cpp
/// @brief Benchmark: RRT vs RRT* vs A* on every scenario, and brute-force vs k-d tree
/// nearest-neighbour search. Prints Markdown tables ready to paste into the README.
///
/// Usage:
///   amp_bench [--scenarios DIR] [--seeds N] [--out FILE] [--check [--iterations N]] [--skip-nn]
///
///   --check    Only diagnose each scenario (start/goal validity, one run per planner).
///   --seeds N  Seeds per sampling planner (default 50). A* is deterministic: one run.
///   --out FILE Also write the Markdown report to FILE.
#include "amp/astar.hpp"
#include "amp/cspace.hpp"
#include "amp/cspace_grid.hpp"
#include "amp/nearest_neighbor.hpp"
#include "amp/planner.hpp"
#include "amp/rng.hpp"
#include "amp/scenario.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef AMP_SCENARIO_DIR
#define AMP_SCENARIO_DIR "scenarios"
#endif

namespace {

using namespace amp;

struct Options {
    std::filesystem::path scenarios = AMP_SCENARIO_DIR;
    int seeds = 50;
    std::string out;
    bool check = false;
    bool skipNn = false;
    int iterations = 0;  ///< If > 0, overrides every scenario's max_iterations.
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value after " << a << "\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--scenarios") {
            o.scenarios = next();
        } else if (a == "--seeds") {
            o.seeds = std::max(1, std::stoi(next()));
        } else if (a == "--out") {
            o.out = next();
        } else if (a == "--check") {
            o.check = true;
        } else if (a == "--iterations") {
            o.iterations = std::stoi(next());
        } else if (a == "--skip-nn") {
            o.skipNn = true;
        } else {
            std::cerr << "usage: amp_bench [--scenarios DIR] [--seeds N] [--out FILE] [--check] [--skip-nn]\n";
            std::exit(a == "--help" ? 0 : 2);
        }
    }
    return o;
}

double median(std::vector<double> v) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

std::string fixed(double v, int decimals) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(decimals) << v;
    return s.str();
}

std::string describe(const CollisionReport& r) {
    std::string s = toString(r.kind);
    if (r.link >= 0) {
        s += " link " + std::to_string(r.link);
    }
    if (r.other >= 0) {
        s += (r.kind == CollisionKind::Self ? " vs link " : " vs obstacle ") + std::to_string(r.other);
    }
    return s;
}

/// ASCII picture of a 2-DOF C-space: '.' free, '#' obstacle, 'b' boundary, 's' self, ':' limit,
/// 'S' start, 'G' goal. theta1 runs left to right, theta2 bottom to top, both over [-pi, pi).
void printAsciiCSpace(const CollisionChecker& checker, const Config& start, const Config& goal) {
    constexpr int kCols = 96;
    constexpr int kRows = 48;
    const CSpaceGrid2D grid = computeCSpaceGrid2D(checker, kCols);
    const auto cellOf = [](double angle, int n) {
        return std::clamp(static_cast<int>((angle + kPi) / kTwoPi * n), 0, n - 1);
    };
    for (int row = kRows - 1; row >= 0; --row) {
        std::string line = "   ";
        const int j = row * kCols / kRows;
        for (int i = 0; i < kCols; ++i) {
            char c = '.';
            switch (grid.at(i, j)) {
                case CollisionKind::ObstacleContact:
                    c = '#';
                    break;
                case CollisionKind::Boundary:
                    c = 'b';
                    break;
                case CollisionKind::Self:
                    c = 's';
                    break;
                case CollisionKind::JointLimit:
                    c = ':';
                    break;
                case CollisionKind::None:
                    break;
            }
            if (cellOf(start[0], kCols) == i && cellOf(start[1], kRows) == row)
                c = 'S';
            if (cellOf(goal[0], kCols) == i && cellOf(goal[1], kRows) == row)
                c = 'G';
            line += c;
        }
        std::cout << line << '\n';
    }
}

// ---------------------------------------------------------------------------
// --check: one-shot diagnosis of every scenario
// ---------------------------------------------------------------------------

int runCheck(const Options& opt) {
    int problems = 0;
    for (const auto& file : listScenarioFiles(opt.scenarios)) {
        std::cout << "== " << file.filename().string() << "\n";
        try {
            const Scenario s = loadScenario(file);
            const CollisionChecker checker(s.arm, s.workspace, s.collision);
            const CollisionReport rs = checker.check(s.start);
            const CollisionReport rg = checker.check(s.goal);
            std::cout << "   start: " << describe(rs) << "\n   goal:  " << describe(rg) << "\n";
            problems += rs.ok() ? 0 : 1;
            problems += rg.ok() ? 0 : 1;
            if (s.arm.dof() == 2) {
                printAsciiCSpace(checker, s.start, s.goal);
            }
            for (PlannerType t : {PlannerType::Rrt, PlannerType::RrtStar, PlannerType::AStar}) {
                PlannerParams p = s.planner;
                p.stopAtFirstSolution = t == PlannerType::RrtStar;
                if (opt.iterations > 0) {
                    p.maxIterations = opt.iterations;
                }
                const PlanResult r = makePlanner(t, checker)->plan(s.start, s.goal, p);
                std::cout << "   " << std::left << std::setw(5) << toString(t) << (r.success ? "OK  " : "FAIL")
                          << "  iters=" << r.iterations << "  nodes=" << r.nodes
                          << "  time=" << fixed(r.planningTimeMs, 1) << " ms  cost=" << fixed(r.pathCost, 3) << "  "
                          << r.message << "\n";
            }
        } catch (const std::exception& e) {
            std::cout << "   ERROR: " << e.what() << "\n";
            ++problems;
        }
    }
    return problems == 0 ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Planner comparison
// ---------------------------------------------------------------------------

std::string plannerTable(const Options& opt) {
    std::ostringstream md;
    md << "| Scenario | DOF | Planner | Success | Median time (ms) | Median cost (rad) | Median nodes |\n"
       << "|---|---:|---|---:|---:|---:|---:|\n";
    for (const auto& file : listScenarioFiles(opt.scenarios)) {
        const Scenario s = loadScenario(file);
        const CollisionChecker checker(s.arm, s.workspace, s.collision);
        std::cerr << "benchmarking " << s.name << "...\n";
        for (PlannerType t : {PlannerType::Rrt, PlannerType::RrtStar, PlannerType::AStar}) {
            const auto planner = makePlanner(t, checker);
            const int runs = t == PlannerType::AStar ? 1 : opt.seeds;  // A* ignores the seed
            std::vector<double> times, costs, nodes;
            int successes = 0;
            std::string note;
            for (int k = 0; k < runs; ++k) {
                PlannerParams p = s.planner;
                p.seed = static_cast<std::uint64_t>(k + 1);
                const PlanResult r = planner->plan(s.start, s.goal, p);
                times.push_back(r.planningTimeMs);
                nodes.push_back(static_cast<double>(r.nodes));
                if (r.success) {
                    ++successes;
                    costs.push_back(r.pathCost);
                } else {
                    note = r.message;
                }
            }
            const bool refused = t == PlannerType::AStar && s.arm.dof() > AStarPlanner::kMaxDof;
            md << "| " << s.name << " | " << s.arm.dof() << " | " << toString(t) << " | ";
            if (refused) {
                md << "refused (" << s.arm.dof() << " DOF) | - | - | - |\n";
                continue;
            }
            md << successes << "/" << runs << " | " << fixed(median(times), 1) << " | "
               << (costs.empty() ? std::string("-") : fixed(median(costs), 3)) << " | " << fixed(median(nodes), 0)
               << " |\n";
        }
    }
    return md.str();
}

// ---------------------------------------------------------------------------
// Nearest-neighbour comparison
// ---------------------------------------------------------------------------

std::string nnTable() {
    std::ostringstream md;
    md << "| DOF | Points | Brute force (µs/query) | k-d tree (µs/query) | Speed-up |\n"
       << "|---:|---:|---:|---:|---:|\n";
    constexpr int kQueries = 2000;
    for (int dof : {2, 3, 6}) {
        const ArmModel arm = ArmModel::uniform(dof, 1.0 / dof, 2.9);
        for (int n : {1000, 10000, 50000}) {
            Rng rng(123);
            BruteForceNN brute(arm);
            KdTreeNN kd(arm);
            for (int i = 0; i < n; ++i) {
                const Config q = sampleUniform(arm, rng);
                brute.add(q);
                kd.add(q);
            }
            std::vector<Config> queries;
            for (int i = 0; i < kQueries; ++i) {
                queries.push_back(sampleUniform(arm, rng));
            }
            const auto timeIt = [&](const NearestNeighbor& index) {
                std::size_t sink = 0;  // keeps the optimiser from removing the loop
                const auto t0 = std::chrono::steady_clock::now();
                for (const Config& q : queries) {
                    sink += index.nearest(q);
                }
                const auto t1 = std::chrono::steady_clock::now();
                if (sink == static_cast<std::size_t>(-1)) {
                    std::cerr << "";
                }
                return std::chrono::duration<double, std::micro>(t1 - t0).count() / kQueries;
            };
            const double tb = timeIt(brute);
            const double tk = timeIt(kd);
            md << "| " << dof << " | " << n << " | " << fixed(tb, 2) << " | " << fixed(tk, 2) << " | "
               << fixed(tb / tk, 1) << "x |\n";
        }
    }
    return md.str();
}

}  // namespace

int main(int argc, char** argv) {
    const Options opt = parseArgs(argc, argv);
    if (listScenarioFiles(opt.scenarios).empty()) {
        std::cerr << "no scenarios found in " << opt.scenarios.string() << "\n";
        return 2;
    }
    if (opt.check) {
        return runCheck(opt);
    }

    std::ostringstream report;
    report << "### Planner comparison (" << opt.seeds << " seeds per sampling planner; A* is deterministic)\n\n"
           << plannerTable(opt) << "\n"
           << "Cost is the C-space path length in radians, before smoothing. Times are wall-clock on the machine "
              "that ran the benchmark, for a Release build.\n";
    if (!opt.skipNn) {
        report << "\n### Nearest-neighbour search (2000 queries, uniformly random points)\n\n" << nnTable();
    }

    std::cout << report.str();
    if (!opt.out.empty()) {
        std::ofstream(opt.out) << report.str();
        std::cerr << "wrote " << opt.out << "\n";
    }
    return 0;
}
