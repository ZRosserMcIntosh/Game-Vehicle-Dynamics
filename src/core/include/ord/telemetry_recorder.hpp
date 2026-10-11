// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Fixed-capacity telemetry ring buffer. All memory is allocated up front, so
// push() is safe to call every physics step (including on the physics thread).
// Dump to CSV from any thread *after* stopping pushes, or copy frames out.
#pragma once

#include <ostream>
#include <vector>

#include "ord/telemetry.hpp"

namespace ord {

class TelemetryRecorder {
public:
    explicit TelemetryRecorder(std::size_t capacity = 240 * 30) : frames_(capacity) {}

    void push(const TelemetryFrame& f) {
        if (frames_.empty()) return;
        frames_[head_] = f;
        head_ = (head_ + 1) % frames_.size();
        if (size_ < frames_.size()) ++size_;
    }
    void clear() { head_ = size_ = 0; }
    std::size_t size() const { return size_; }
    std::size_t capacity() const { return frames_.size(); }
    // 0 = oldest retained frame.
    const TelemetryFrame& frame(std::size_t i) const {
        const std::size_t start = (head_ + frames_.size() - size_) % frames_.size();
        return frames_[(start + i) % frames_.size()];
    }
    void writeCsv(std::ostream& os) const {
        if (size_ == 0) return;
        writeCsvHeader(os, frame(0).wheelCount);
        for (std::size_t i = 0; i < size_; ++i) writeCsvRow(os, frame(i));
    }

private:
    std::vector<TelemetryFrame> frames_;
    std::size_t head_ = 0;
    std::size_t size_ = 0;
};

}  // namespace ord
