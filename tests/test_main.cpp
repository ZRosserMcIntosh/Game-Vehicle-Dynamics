// OpenRoad Dynamics — test runner.
// SPDX-License-Identifier: MIT
#include <cstring>

#include "ord_test.hpp"

int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int run = 0, failedCases = 0;
    for (const auto& c : ordtest::registry()) {
        if (filter && !std::strstr(c.name.c_str(), filter)) continue;
        const int before = ordtest::failures();
        std::cout << "[ RUN  ] " << c.name << std::endl;
        c.fn();
        ++run;
        if (ordtest::failures() != before) {
            ++failedCases;
            std::cout << "[ FAIL ] " << c.name << std::endl;
        } else {
            std::cout << "[  OK  ] " << c.name << std::endl;
        }
    }
    std::cout << run - failedCases << "/" << run << " test cases passed\n";
    return failedCases == 0 ? 0 : 1;
}
