#include "dofc/io/CsvLogger.hpp"

#include <iomanip>
#include <stdexcept>

namespace dofc {

CsvLogger::CsvLogger(const std::string& path, const std::vector<std::string>& header)
    : file_(path) {
  if (!file_) {
    throw std::runtime_error("failed to open CSV file: " + path);
  }

  for (std::size_t i = 0; i < header.size(); ++i) {
    if (i > 0) {
      file_ << ',';
    }
    file_ << header[i];
  }
  file_ << '\n';
}

void CsvLogger::writeRow(const std::vector<double>& values) {
  file_ << std::setprecision(10);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      file_ << ',';
    }
    file_ << values[i];
  }
  file_ << '\n';
}

bool CsvLogger::good() const noexcept {
  return file_.good();
}

}  // namespace dofc
