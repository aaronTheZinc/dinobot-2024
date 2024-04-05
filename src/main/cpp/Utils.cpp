#include <Utils.h>


double Utils::linearScale(double x, double minVal = -1.0, double maxVal = 1.0) {
  // Handle case where min and max are equal
  if (minVal == maxVal) {
    return 0.0;
  }

  // Calculate normalization factor
  const double normFactor = 2.0 / (maxVal - minVal);

  // Apply linear scaling with clamping
  return std::max(-1.0, std::min(1.0, normFactor * (x - minVal)));
}