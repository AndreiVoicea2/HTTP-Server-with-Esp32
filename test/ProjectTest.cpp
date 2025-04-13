#ifdef UNIT_TEST

#include <Arduino.h>
#include <unity.h>
#include "formulas.h"

void test_calculateDewPoint() {
  float T = 25.0;
  float RH = 60.0;
  float dp = calculateDewPoint(T, RH);
 
  TEST_ASSERT_FLOAT_WITHIN(0.5, 16.7, dp);
}


void test_calculateHeatIndex() {
  float T = 30.0;
  float RH = 70.0;
  float hi = calculateHeatIndex(T, RH);

  TEST_ASSERT_FLOAT_WITHIN(2.0, 35.0, hi);
}


void test_calculateAbsoluteHumidity() {
  float T = 25.0;
  float RH = 60.0;
  float absHum = calculateAbsoluteHumidity(T, RH);

  TEST_ASSERT_FLOAT_WITHIN(2.0, 12.0, absHum);
}


void test_calculateSaturationVaporPressure() {
  float T = 25.0;
  float svp = calculateSaturationVaporPressure(T);

  TEST_ASSERT_FLOAT_WITHIN(1.0, 31.7, svp);
}


void test_calculateHumidex() {
  float T = 30.0;
  float dewPoint = 25.0; 
  float humidex = calculateHumidex(T, dewPoint);

  TEST_ASSERT_FLOAT_WITHIN(0.5, 42.0, humidex);
}


void test_calculateWetBulbTemperature() {
  float T = 30.0;
  float RH = 70.0;
  float wb = calculateWetBulbTemperature(T, RH);

  TEST_ASSERT_FLOAT_WITHIN(2.0, 27.0, wb);
}

void runTests() {
  UNITY_BEGIN();
  RUN_TEST(test_calculateDewPoint);
  RUN_TEST(test_calculateHeatIndex);
  RUN_TEST(test_calculateAbsoluteHumidity);
  RUN_TEST(test_calculateSaturationVaporPressure);
  RUN_TEST(test_calculateHumidex);
  RUN_TEST(test_calculateWetBulbTemperature);
  UNITY_END();
}

void setup() {

  delay(2000);
  Serial.begin(115200);
  runTests();
}

void loop() {

}

#endif  