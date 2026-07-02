#pragma once

#include <fstream>
#include <string>
#include <vector>

namespace dofc {

class CsvLogger {
public:
  CsvLogger(const std::string& path, const std::vector<std::string>& header);

  void writeRow(const std::vector<double>& values);
  bool good() const noexcept;

private:
  std::ofstream file_;
};

}  // namespace dofc
