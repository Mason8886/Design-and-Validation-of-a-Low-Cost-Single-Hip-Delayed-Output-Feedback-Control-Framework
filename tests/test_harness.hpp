#pragma once

#include <cmath>
#include <iostream>
#include <string>

namespace test {

inline int failures = 0;

inline void expectTrue(bool condition, const std::string& message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

inline void expectNear(double actual,
                       double expected,
                       double tolerance,
                       const std::string& message) {
  if (std::abs(actual - expected) > tolerance) {
    ++failures;
    std::cerr << "FAIL: " << message << " actual=" << actual
              << " expected=" << expected << " tolerance=" << tolerance << '\n';
  }
}

}  // namespace test
