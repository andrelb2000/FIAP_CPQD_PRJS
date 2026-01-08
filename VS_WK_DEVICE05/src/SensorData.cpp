#include "SensorData.h"

SensorData::SensorData() {}
SensorData::SensorData(const String &lbl) : label(lbl) {}

void SensorData::add(double temp, double gas) {
  temps.push_back(temp);
  gasLevels.push_back(gas);
}

double SensorData::avgTemp() const {
  if (temps.empty()) return 0.0;
  double s = 0.0;
  for (double v : temps) s += v;
  return s / (double)temps.size();
}

double SensorData::avgGas() const {
  if (gasLevels.empty()) return 0.0;
  double s = 0.0;
  for (double v : gasLevels) s += v;
  return s / (double)gasLevels.size();
}
void SensorData::clear() {
  temps.clear();
  gasLevels.clear();
} 