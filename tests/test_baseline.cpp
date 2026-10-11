// OpenRoad Dynamics — golden baseline guard.
// SPDX-License-Identifier: MIT
//
// Fails when any key benchmark drifts from benchmarks/baseline.csv beyond its
// tolerance. Physics changes are welcome, but they must update the baseline
// explicitly (`ordbench --write-baseline benchmarks/baseline.csv`) so the change
// is visible and justified in review rather than silently absorbed.
#include <fstream>
#include <map>
#include <sstream>

#include "ord_test.hpp"
#include "scenarios.hpp"

#ifndef ORD_SOURCE_DIR
#error "ORD_SOURCE_DIR must be defined"
#endif

TEST_CASE("baseline: key benchmark metrics match the committed golden values") {
    std::ifstream in(std::string(ORD_SOURCE_DIR) + "/benchmarks/baseline.csv");
    CHECK(in.good());
    std::map<std::string, std::pair<double, double>> golden;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#' || line.rfind("metric,", 0) == 0) continue;
        std::stringstream ss(line);
        std::string name, v, t;
        std::getline(ss, name, ',');
        std::getline(ss, v, ',');
        std::getline(ss, t, ',');
        golden[name] = {std::stod(v), std::stod(t)};
    }
    const auto metrics = ord::scenarios::computeBaselineMetrics();
    CHECK(golden.size() == metrics.size());
    for (const auto& m : metrics) {
        const auto it = golden.find(m.name);
        if (it == golden.end()) {
            std::cerr << "  missing from baseline: " << m.name << "\n";
            CHECK(false);
            continue;
        }
        const double ref = it->second.first, tol = it->second.second;
        if (std::abs(m.value - ref) > tol * std::abs(ref)) {
            std::cerr << "  " << m.name << ": " << m.value << " vs golden " << ref << " (tol " << tol * 100 << "%)\n";
            CHECK(false);
        }
    }
}
