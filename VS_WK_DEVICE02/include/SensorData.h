// SensorData.h
#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <Arduino.h>
#include <vector>

class SensorData {
public:
  String label;
  std::vector<double> temps;
  std::vector<double> gasLevels;

  SensorData();
  SensorData(const String &lbl);

  // Adiciona um único par (temperatura, gas) aos vetores internos.
  void add(double temp, double gas);

  // Retorna a média das temperaturas; 0.0 se vazio.
  double avgTemp() const;

  // Retorna a média dos níveis de gás; 0.0 se vazio.
  double avgGas() const;

  void clear();
};

#endif // SENSORDATA_H
