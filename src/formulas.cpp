#include <math.h>
#include "formulas.h"

#define DEW_POINT_COEFF_A 17.27
#define DEW_POINT_COEFF_B 237.7

#define HEAT_INDEX_CONST -42.379
#define HEAT_INDEX_TEMP_FACTOR 2.04901523
#define HEAT_INDEX_HUM_FACTOR 10.14333127
#define HEAT_INDEX_TEMP_HUM_MUL -0.22475541
#define HEAT_INDEX_TEMP_SQR -0.00683783
#define HEAT_INDEX_HUM_SQR -0.05481717
#define HEAT_INDEX_TEMP_SQR_HUM 0.00122874
#define HEAT_INDEX_TEMP_HUM_SQR 0.00085282
#define HEAT_INDEX_TEMP_SQR_HUM_SQR -0.00000199

#define SATURATION_EXP_COEFF 17.67
#define SATURATION_DENOMINATOR 243.5
#define SATURATION_BASE_PRESSURE 6.112
#define ABSOLUTE_HUM_COEFF 2.1674
#define CELSIUS_TO_KELVIN 273.15

#define HUMIDEX_COEFFICIENT 0.5555
#define HUMIDEX_REFERENCE 10.0

#define WET_BULB_ATAN_COEFF 0.151977
#define WET_BULB_HUM_OFFSET 8.313659
#define WET_BULB_ATAN_SUBTRACT 1.676331
#define WET_BULB_POW_COEFF 0.00391838
#define WET_BULB_ATAN_HUM_COEFF 0.023101
#define WET_BULB_CONSTANT 4.686035

float calculateDewPoint(float temperature, float humidity)
{
  float alpha = log(humidity / 100.0) + (DEW_POINT_COEFF_A * temperature) / (DEW_POINT_COEFF_B + temperature);
  float dewPoint = (DEW_POINT_COEFF_B * alpha) / (DEW_POINT_COEFF_A - alpha);
  return dewPoint;
}

/**
 * This function converts temperature to Fahrenheit for the computation and then
 * converts the result back to Celsius.
 **/
float calculateHeatIndex(float temperatureC, float humidity) 
{
  float temperatureF = temperatureC * 9.0 / 5.0 + 32;
  float HI_F = HEAT_INDEX_CONST 
               + HEAT_INDEX_TEMP_FACTOR * temperatureF 
               + HEAT_INDEX_HUM_FACTOR * humidity 
               + HEAT_INDEX_TEMP_HUM_MUL * temperatureF * humidity 
               + HEAT_INDEX_TEMP_SQR * temperatureF * temperatureF 
               + HEAT_INDEX_HUM_SQR * humidity * humidity 
               + HEAT_INDEX_TEMP_SQR_HUM * temperatureF * temperatureF * humidity 
               + HEAT_INDEX_TEMP_HUM_SQR * temperatureF * humidity * humidity 
               + HEAT_INDEX_TEMP_SQR_HUM_SQR * temperatureF * temperatureF * humidity * humidity;
  float HI_C = (HI_F - 32) * 5.0 / 9.0;
  return HI_C;
}

float calculateAbsoluteHumidity(float temperatureC, float humidity)
{
  float exponent = (SATURATION_EXP_COEFF * temperatureC) / (temperatureC + SATURATION_DENOMINATOR);
  float saturationVaporPressure = SATURATION_BASE_PRESSURE * exp(exponent);
  float absoluteHumidity = (saturationVaporPressure * humidity * ABSOLUTE_HUM_COEFF) / (temperatureC + CELSIUS_TO_KELVIN);
  return absoluteHumidity;
}

float calculateSaturationVaporPressure(float temperatureC)
{
  float exponent = (SATURATION_EXP_COEFF * temperatureC) / (temperatureC + SATURATION_DENOMINATOR);
  return SATURATION_BASE_PRESSURE * exp(exponent);
}

float calculateHumidex(float temperatureC, float dewPointC)
{
  float e = SATURATION_BASE_PRESSURE * exp((SATURATION_EXP_COEFF * dewPointC) / (dewPointC + SATURATION_DENOMINATOR));
  float humidex = temperatureC + HUMIDEX_COEFFICIENT * (e - HUMIDEX_REFERENCE);
  return humidex;
}

/**
 * Wet bulb temperature is a measure of the lowest temperature that can be reached
 * solely by evaporative cooling.
 **/
float calculateWetBulbTemperature(float temperatureC, float humidity)
{
  float wetBulb = temperatureC * atan(WET_BULB_ATAN_COEFF * sqrt(humidity + WET_BULB_HUM_OFFSET)) + atan(temperatureC + humidity) - atan(humidity - WET_BULB_ATAN_SUBTRACT) + WET_BULB_POW_COEFF * pow(humidity, 1.5) * atan(WET_BULB_ATAN_HUM_COEFF * humidity) - WET_BULB_CONSTANT;
  return wetBulb;
}