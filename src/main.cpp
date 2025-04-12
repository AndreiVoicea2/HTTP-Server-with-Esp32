/**
 * This project reads data from a DHT11 sensor, computes various environmental
 * parameters, and serves a dynamic HTML page over WiFi. The web page template is
 * stored in SPIFFS. Additionally, the LCD module displays connection status.
 *
 * Made by: Andrei Voicea
 * Contact: andreivoicea7@gmail.com
 **/

#include "DHT.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include "credentials.h"
#include <LiquidCrystal_I2C.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SPIFFS.h"

#define DHTPIN 4
#define DHTTYPE DHT11
#define BUZZER 12
#define ALARM_LIMIT 26
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define BUZZER_FRQ 1000
#define BUZZER_TIME 1000
#define CONNECT_DLY 500

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);

#pragma region Formulas

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

#pragma endregion

#pragma region Server_Functions

String readHTMLFile(const char *path)
{
  File file = SPIFFS.open(path, "r");
  if (!file)
  {
    Serial.println("Can't open HTML");
    return "";
  }
  String html = file.readString();
  file.close();
  return html;
}

/**
 * Reads sensor data, processes various calculations, and serves a dynamic HTML page
 * with the updated values. Also triggers the buzzer if the heat index exceeds the threshold.
 **/
void handleRoot()
{
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float HeatIndex = calculateHeatIndex(temperature, humidity);

  if (isnan(temperature) || isnan(humidity))
  {
    server.send(500, "text/plain", "Error: failed to read from sensor");
    return;
  }

  String html = readHTMLFile("/index.html");
  html.replace("{temperature}", String(temperature));
  html.replace("{humidity}", String(humidity));
  html.replace("{dewPoint}", String(calculateDewPoint(temperature, humidity)));
  html.replace("{heatIndex}", String(HeatIndex));
  html.replace("{absoluteHumidity}", String(calculateAbsoluteHumidity(temperature, humidity)));
  html.replace("{saturationVaporPressure}", String(calculateSaturationVaporPressure(temperature)));
  html.replace("{humidex}", String(calculateHumidex(temperature, humidity)));
  html.replace("{wetBulb}", String(calculateWetBulbTemperature(temperature, humidity)));

  if (HeatIndex >= ALARM_LIMIT)
    tone(BUZZER, BUZZER_FRQ, BUZZER_TIME);

  server.send(200, "text/html", html);
}

/**
 * Task that continuously checks for a WiFi connection and initializes the server.
 *
 * This FreeRTOS task attempts to connect to the WiFi network; once connected, it starts the web server
 * and displays status information on the LCD.
 **/
void Connect(void *parameter)
{
  for (;;)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED)
      {
        lcd.clear();
        lcd.print("Connecting");
        delay(CONNECT_DLY);
        lcd.print(".");
        delay(CONNECT_DLY);
        lcd.print(".");
        delay(CONNECT_DLY);
        lcd.print(".");
        delay(CONNECT_DLY);
      }

      tone(BUZZER, BUZZER_FRQ, BUZZER_TIME);
      Serial.println(WiFi.localIP());
      lcd.setCursor(0, 0);
      lcd.print("Server Started");
      server.on("/", handleRoot);
      server.begin();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

#pragma endregion

void setup()
{
  Serial.begin(115200);
  dht.begin();
  lcd.init();
  lcd.backlight();

  if (!SPIFFS.begin(true))
  {
    Serial.println("Error mounting SPIFFS");
    return;
  }

  xTaskCreate(Connect, "ConnectTask", 4096, NULL, 1, NULL);
}

void loop()
{
  server.handleClient();
}
