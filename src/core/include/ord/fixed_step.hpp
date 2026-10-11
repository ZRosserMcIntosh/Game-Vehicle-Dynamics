// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Fixed-timestep accumulator. Physics always advances in exact multiples of
// `dt`, independent of render frame rate, which is required for
// reproducibility (and, later, lockstep/rollback networking).
#pragma once

#include "ord/math.hpp"

namespace ord {

class FixedStepper {
public:
    explicit FixedStepper(Real hz, int maxStepsPerFrame = 16) : dt_(1.0 / hz), maxSteps_(maxStepsPerFrame) {}

    // Returns the number of fixed steps to run for this frame. Excess time
    // beyond maxStepsPerFrame is dropped (prevents the "spiral of death").
    int advance(Real frameSeconds) {
        acc_ += std::max(0.0, frameSeconds);
        int n = static_cast<int>(acc_ / dt_);
        if (n > maxSteps_) {
            n = maxSteps_;
            acc_ = 0;
        } else {
            acc_ -= n * dt_;
        }
        return n;
    }
    // Fraction of a step left over, for render interpolation.
    Real alpha() const { return acc_ / dt_; }
    Real dt() const { return dt_; }

private:
    Real dt_;
    int maxSteps_;
    Real acc_ = 0;
};

}  // namespace ord
