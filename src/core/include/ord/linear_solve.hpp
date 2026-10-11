// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Fixed-capacity dense linear solver (Gaussian elimination, partial pivoting).
// Used for the small drivetrain/wheel systems (typically < 20 unknowns).
// No heap allocation so it is safe to call from the physics step.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

#include "ord/math.hpp"

namespace ord {

template <std::size_t Capacity>
class DenseSystem {
public:
    void reset(std::size_t n) {
        n_ = n;
        for (std::size_t i = 0; i < n; ++i) {
            b_[i] = 0;
            for (std::size_t j = 0; j < n; ++j) a_[i][j] = 0;
        }
    }
    std::size_t size() const { return n_; }
    Real& a(std::size_t i, std::size_t j) { return a_[i][j]; }
    Real& b(std::size_t i) { return b_[i]; }

    // Solves in place; the solution is returned in x(). Returns false if singular.
    bool solve() {
        for (std::size_t col = 0; col < n_; ++col) {
            std::size_t piv = col;
            Real best = std::abs(a_[col][col]);
            for (std::size_t r = col + 1; r < n_; ++r)
                if (std::abs(a_[r][col]) > best) { best = std::abs(a_[r][col]); piv = r; }
            if (best < 1e-14) return false;
            if (piv != col) {
                std::swap(a_[piv], a_[col]);
                std::swap(b_[piv], b_[col]);
            }
            for (std::size_t r = col + 1; r < n_; ++r) {
                const Real f = a_[r][col] / a_[col][col];
                if (f == 0) continue;
                for (std::size_t c = col; c < n_; ++c) a_[r][c] -= f * a_[col][c];
                b_[r] -= f * b_[col];
            }
        }
        for (std::size_t i = n_; i-- > 0;) {
            Real s = b_[i];
            for (std::size_t j = i + 1; j < n_; ++j) s -= a_[i][j] * x_[j];
            x_[i] = s / a_[i][i];
        }
        return true;
    }
    Real x(std::size_t i) const { return x_[i]; }

private:
    std::size_t n_ = 0;
    std::array<std::array<Real, Capacity>, Capacity> a_{};
    std::array<Real, Capacity> b_{};
    std::array<Real, Capacity> x_{};
};

}  // namespace ord
