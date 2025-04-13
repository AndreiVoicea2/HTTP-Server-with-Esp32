/**
 * This project reads data from a DHT11 sensor, computes various environmental
 * parameters, and serves a dynamic HTML page over WiFi. The web page template is
 * stored in SPIFFS. Additionally, the LCD module displays connection status.
 *
 * Made by: Andrei Voicea
 * Project Location: https://github.com/AndreiVoicea2
 **/


#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <LiquidCrystal_I2C.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "SPIFFS.h"
#include "DHT.h"
#include "credentials.h"
#include "formulas.h"


#define BAUD_RATE 115200
#define DHTPIN 4
#define DHTTYPE DHT11
#define LEDC_FRQ 2000
#define LEDC_RES 8
#define BUZZER_PIN 12
#define BUZZER_CHANNEL 0
#define BUZZER_FRQ 1000
#define BUZZER_TIME 1000
#define ALARM_LIMIT 26
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define CONNECT_DLY 500
#define TASK_DLY 1000
#define STACK_SIZE 4096
#define PRIORITY 1

DHT dht(DHTPIN, DHTTYPE);
WebServer server(80);
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);


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
    tone(BUZZER_PIN, BUZZER_FRQ, BUZZER_TIME);

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

      tone(BUZZER_PIN, BUZZER_FRQ, BUZZER_TIME);
      Serial.println(WiFi.localIP());
      lcd.setCursor(0, 0);
      lcd.print("Server Started");
      server.on("/", handleRoot);
      server.begin();
    }

    vTaskDelay(TASK_DLY / portTICK_PERIOD_MS);
  }
}

#pragma endregion

void setup()
{
  Serial.begin(BAUD_RATE);
  dht.begin();
  lcd.init();
  lcd.backlight();
  ledcSetup(BUZZER_CHANNEL, LEDC_FRQ, LEDC_RES);         
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);      

  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Error");
    return;
  }

  xTaskCreate(Connect, "ConnectTask", STACK_SIZE, NULL, PRIORITY, NULL);
}

void loop()
{
  server.handleClient();
}
