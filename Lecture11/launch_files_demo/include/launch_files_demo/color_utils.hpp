#pragma once
#include <string>

namespace launch_demo_nodes {

class Color {
public:
  static constexpr const char *RED = "\033[1;31m";
  static constexpr const char *GREEN = "\033[1;32m";
  static constexpr const char *YELLOW = "\033[1;33m";
  static constexpr const char *BLUE = "\033[1;34m";
  static constexpr const char *PURPLE = "\033[35m";
  static constexpr const char *CYAN = "\033[36m";
  static constexpr const char *RESET = "\033[0m";
};

} // namespace launch_demo_nodes