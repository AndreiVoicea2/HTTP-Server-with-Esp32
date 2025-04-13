#ifndef FORMULAS_H_
#define FORMULAS_H_

float calculateDewPoint(float temperature, float humidity);
float calculateHeatIndex(float temperatureC, float humidity);
float calculateAbsoluteHumidity(float temperatureC, float humidity);
float calculateSaturationVaporPressure(float temperatureC);
float calculateHumidex(float temperatureC, float dewPointC);
float calculateWetBulbTemperature(float temperatureC, float humidity);

#endif