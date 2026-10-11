// OpenRoad Dynamics — minimal dependency-free test harness.
// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace ordtest {

struct Case {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}
inline int& failures() {
    static int f = 0;
    return f;
}
struct Registrar {
    Registrar(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

// Long-running scenarios run at full length in optimised builds and shortened
// (same invariants, fewer simulated seconds) in Debug/sanitizer builds.
inline constexpr bool kOptimisedBuild =
#ifdef NDEBUG
    true;
#else
    false;
#endif

}  // namespace ordtest

#define ORD_CAT2(a, b) a##b
#define ORD_CAT(a, b) ORD_CAT2(a, b)
#define TEST_CASE(name)                                                              \
    static void ORD_CAT(ord_test_fn_, __LINE__)();                                   \
    static ordtest::Registrar ORD_CAT(ord_test_reg_, __LINE__)(name, &ORD_CAT(ord_test_fn_, __LINE__)); \
    static void ORD_CAT(ord_test_fn_, __LINE__)()

#define CHECK(cond)                                                                            \
    do {                                                                                       \
        if (!(cond)) {                                                                         \
            ++ordtest::failures();                                                             \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #cond << "\n";     \
        }                                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                          \
    do {                                                                                               \
        const double ord_a_ = (a), ord_b_ = (b), ord_t_ = (tol);                                       \
        if (!(std::abs(ord_a_ - ord_b_) <= ord_t_)) {                                                  \
            ++ordtest::failures();                                                                     \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": |" #a " - " #b "| <= " #tol  \
                      << "  (" << ord_a_ << " vs " << ord_b_ << ")\n";                                 \
        }                                                                                              \
    } while (0)

#define CHECK_RANGE(v, lo, hi)                                                                         \
    do {                                                                                               \
        const double ord_v_ = (v);                                                                     \
        if (!(ord_v_ >= (lo) && ord_v_ <= (hi))) {                                                     \
            ++ordtest::failures();                                                                     \
            std::cerr << "  FAILED " << __FILE__ << ":" << __LINE__ << ": " #v " = " << ord_v_         \
                      << " not in [" << (lo) << ", " << (hi) << "]\n";                                 \
        }                                                                                              \
    } while (0)
