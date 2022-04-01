#include <LoRa.h>

#include <TinyGPS++.h>

#include <SPI.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "IOSdcard.h"
#include "BMI088.h"
#include <BMP388_DEV.h>

static const unsigned char PROGMEM bitmap[]{
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFB, 0x79, 0xEF, 0xBB, 0xCF, 0x37, 0xDF, 0xBB, 0xED, 0x9F, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0x7F, 0xBF, 0xEF, 0xA5, 0xFF, 0xCF, 0x77, 0xDF, 0x7B, 0xA9, 0xFF, 0xF7, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0x5F, 0xFF, 0xA7, 0xEF, 0x57, 0xDF, 0x77, 0xED,
    0x4F, 0xF7, 0x5F, 0xFB, 0xC9, 0xBF, 0xD8, 0x70, 0x50, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0xC0, 0xD4, 0x7E, 0x56, 0xDC, 0x76, 0xDF, 0x73, 0xCF, 0xDF, 0x73, 0xDF, 0x7B,
    0xE7, 0x7F, 0xDB, 0x5B, 0xEF, 0xA7, 0xFF, 0x5B, 0xF5, 0x4F, 0xFB, 0xAF, 0xEF, 0x7F, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFB, 0xFE, 0xEF, 0xB5, 0xE7, 0x3D, 0xF6, 0x56, 0xFB, 0x9F,
    0xF7, 0x2F, 0xFF, 0x99, 0xE7, 0xBD, 0xDB, 0xBC, 0xDE, 0xF7, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xAF, 0xFA, 0x5F, 0x75, 0xD7, 0x79, 0xEF, 0x34, 0x0F, 0x05, 0x03, 0x01, 0x00,
    0x00, 0x00, 0x02, 0x07, 0x5F, 0xFA, 0xAE, 0xEB, 0x57, 0xDD, 0x77, 0xE8, 0xF0, 0xF0, 0xF8, 0xFC,
    0xFE, 0xFF, 0xFF, 0x7F, 0xBD, 0xEB, 0xAD, 0xF7, 0x7D, 0x55, 0xB7, 0xED, 0xFF, 0xFF, 0x7F, 0xFF,
    0xFF, 0xBF, 0xFF, 0xBF, 0xBF, 0xBD, 0xEF, 0x79, 0x79, 0xDF, 0xB2, 0xCF, 0xF5, 0xDF, 0xFE, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xEF, 0x9A, 0xFF, 0x2C, 0xF7,
    0x75, 0xCF, 0xFA, 0xAE, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xF7, 0x5D, 0xFB, 0x9E, 0xED, 0x35, 0xF7, 0x5D, 0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0xC0, 0xF1, 0x5F, 0xFC, 0x97, 0x59, 0xFF, 0xA5, 0xEF, 0x5E, 0xF7, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0x75, 0x5F, 0xF3, 0xAE, 0x75, 0xCF, 0xFA, 0xAE, 0xFF, 0xFF, 0xFF, 0xFE,
    0xDE, 0xFB, 0x4E, 0xF3, 0x7B, 0xDE, 0xAA, 0xEF, 0xBD, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEC, 0xEF, 0xEA, 0xDF, 0xF9,
    0xDA, 0xFF, 0xE5, 0xF7, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x3F, 0x0F, 0x07, 0x07, 0x41, 0x7E, 0x6B,
    0x6C, 0x37, 0x5D, 0x37, 0x76, 0x1F, 0x02, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC,
    0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xEF, 0xF9, 0xF7, 0xD5, 0xDB, 0xFE, 0xC5, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0xF2, 0xDF, 0xF9, 0xF5, 0xDF, 0xF3, 0xDE, 0xEF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFA, 0xF7, 0xE5, 0xFF, 0x9A, 0xDA, 0xFF, 0xE5, 0xF7, 0xFF, 0xBF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// PINS
#define SDCARDCS 33

// OLED Display
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
SPIClass spi = SPIClass(VSPI);
// Handle tasks
TaskHandle_t AccelAndGyroTask;
TaskHandle_t PressureTask;
TaskHandle_t DisplayTask;
TaskHandle_t RadioTask;
TaskHandle_t FlusherTask;
TaskHandle_t GPSTask;

// Semaphores
SemaphoreHandle_t SDWriteSemaphore = xSemaphoreCreateMutex();
SemaphoreHandle_t I2CSemaphore = xSemaphoreCreateMutex();
SemaphoreHandle_t DisplaySemaphore = xSemaphoreCreateMutex();

// Files
File pressureFile;
File accelGyroFile;
File gpsFile;

// Buffer size for data
#define BUFFERSIZE 5

// Hold Pressure data
struct PressureStuct
{
  float pressure;
  float temperature;
  float altitude;
  unsigned long time;
};

struct GPSStruct
{
  double lat, lng;
  uint32_t date;
  uint32_t time;
  double speed;
  double course;
  double altitude;
  int satellites;
  int hdop;
  unsigned long systemTime;
};

struct AccelerationAndGyroStruct
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  unsigned long time;
};

QueueHandle_t AccelQueue = xQueueCreate(BUFFERSIZE, sizeof(AccelerationAndGyroStruct));
QueueHandle_t PressureQueue = xQueueCreate(BUFFERSIZE, sizeof(PressureStuct));
QueueHandle_t GPSQueue = xQueueCreate(BUFFERSIZE, sizeof(GPSStruct));

QueueHandle_t DisplayAccelQueue = xQueueCreate(1, sizeof(AccelerationAndGyroStruct));
QueueHandle_t DisplayPressureQueue = xQueueCreate(1, sizeof(PressureStuct));
QueueHandle_t DisplayGPSQueue = xQueueCreate(1, sizeof(GPSStruct));

QueueHandle_t RadioAccelQueue = xQueueCreate(1, sizeof(AccelerationAndGyroStruct));
QueueHandle_t RadioPressureQueue = xQueueCreate(1, sizeof(PressureStuct));
QueueHandle_t RadioGPSQueue = xQueueCreate(1, sizeof(GPSStruct));

// Devices
BMP388_DEV bmp388;
SPIClass *hspi;
RF24 radio(26, 15);

void Radio(void *pvParameters)
{
  // HSPI is the hardware SPI port for the radio
  hspi = new SPIClass(HSPI);
  hspi->begin();

  // ADDRESS
  // NSS 26
  // reset -1
  // DI0 16

  LoRa.setSPI(*hspi);
  LoRa.setPins(26,-1,16);
  while(!LoRa.begin(868E6))
  {
    Serial.println(".");
    delay(500);
  }

  LoRa.setSyncWord(0xD3);
  LoRa.setSignalBandwidth(250E3);
  LoRa.beginPacket();
  LoRa.print("Hello ");
  LoRa.print("Hello ");
  LoRa.endPacket();

  while (1)
  {
    // unsigned long start_timer = micros();               // start the timer
    // bool report = radio.write(&payload, sizeof(float)); // transmit & save the report
    // unsigned long end_timer = micros();                 // end the timer

    // if (report)
    // {
    //   Serial.print(F("Transmission successful! ")); // payload was delivered
    //   Serial.print(F("Time to transmit = "));
    //   Serial.print(end_timer - start_timer); // print the timer result
    //   Serial.print(F(" us. Sent: "));
    //   Serial.println(payload); // print payload sent
    //   payload += 0.01;         // increment float payload
    // }
    // else
    // {
    //   Serial.println(F("Transmission failed or timed out")); // payload was not delivered
    // }
    delay(1000);
    LoRa.beginPacket();
  LoRa.print("Hello ");
  LoRa.print("Hello ");
  LoRa.endPacket();

  }
}

void Display(void *pvParameters)
{

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  display.display();
  delay(1000);
  PressureStuct latestPressureData;
  AccelerationAndGyroStruct latestAccelGyroData;
  GPSStruct latestGPSData;
  while (1)
  {
    display.clearDisplay();

    // copy latest data to display

    xQueueReceive(DisplayPressureQueue, &latestPressureData, portMAX_DELAY);
    xQueueReceive(DisplayAccelQueue, &latestAccelGyroData, portMAX_DELAY);
    xQueueReceive(DisplayGPSQueue, &latestGPSData, portMAX_DELAY);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    // display.printf("Altitude: %.4f", latestPressureData.altitude);
    // GPS
    display.printf("Lat: %.2f ", latestGPSData.lat);
    display.printf("Lng: %.2f", latestGPSData.lng);
    display.setCursor(0, 6);
    display.printf("Pressure: %.1f", latestPressureData.pressure);
    display.setCursor(0, 12);
    display.printf("Temperature: %.1f", latestPressureData.temperature);
    display.setCursor(0, 18);
    display.printf("Accel: %.1f, %.1f, %.1f Gyro: %.1f, %.1f, %.1f", latestAccelGyroData.ax / 1000, latestAccelGyroData.ay / 1000, latestAccelGyroData.az / 1000, latestAccelGyroData.gx, latestAccelGyroData.gy, latestAccelGyroData.gz);

    // display.println("Hello World!");
    display.display();
  }
}

void Pressure(void *pvParameters)
{
  if (bmp388.begin())
  {
    Serial.println("BMP388 is online");
  }
  else
  {
    Serial.println("BMP388 is not offline");
  }                                   
         // Default initialisation, place the BMP388 into SLEEP_MODE
  bmp388.setTimeStandby(TIME_STANDBY_160MS); // Set the standby time to 1280ms
  bmp388.startNormalConversion();            // Start NORMAL conversion mode
  float temperature, pressure, altitude;
  uint8_t initial = bmp388.getMeasurements(temperature, pressure, altitude);
  while (!initial)
  {
    initial = bmp388.getMeasurements(temperature, pressure, altitude);
  }
  bmp388.setSeaLevelPressure(pressure);
  while (1)
  {
    if (bmp388.getMeasurements(temperature, pressure, altitude)) // Check if the measurement is complete
    {
      PressureStuct pressureStruct;
      pressureStruct.pressure = pressure;
      pressureStruct.temperature = temperature;
      pressureStruct.altitude = altitude;
      pressureStruct.time = millis();
      xQueueSend(PressureQueue, &pressureStruct, 0);
      xQueueSend(DisplayPressureQueue, &pressureStruct, 0);
      xQueueSend(RadioPressureQueue, &pressureStruct, 0);
    }
    delay(100);
  }
}

void AccelAndGyro(void *pvParameters)
{
  delay(1000);
  if (bmi088.isConnection())
  {
    bmi088.initialize();
    Serial.println("BMI088 connected");
  }
  else
  {
    Serial.println("BMI088 not connected");
    while (1)
    {
    }
  }
  while (1)
  {
    float x, y, z = 0;
    bmi088.getAcceleration(&x, &y, &z);
    // printf("Acceleration: %f, %f, %f\n", x, y, z);
    float xg, yg, zg = 0;
    bmi088.getGyroscope(&xg, &yg, &zg);
    // printf("Gyroscope: %f, %f, %f\n", xg, yg, zg);
    AccelerationAndGyroStruct accelAndGyro;
    accelAndGyro.ax = x;
    accelAndGyro.ay = y;
    accelAndGyro.az = z;
    accelAndGyro.gx = xg;
    accelAndGyro.gy = yg;
    accelAndGyro.gz = zg;
    accelAndGyro.time = millis();
    xQueueSend(AccelQueue, &accelAndGyro, 0);
    xQueueSend(DisplayAccelQueue, &accelAndGyro, 0);
    xQueueSend(RadioAccelQueue, &accelAndGyro, 0);

    delay(100);
  }
}

void GPS(void *pvParameters)
{
  TinyGPSPlus gps;

  while (1)
  {
    GPSStruct gpsStruct = {};
    while (Serial.available() > 0)
    {
      char byte = Serial.read();
      Serial.print(byte);
      gps.encode(byte);
    }
    if (gps.location.isUpdated())
    {
      if (gps.location.isValid())
      {
        gpsStruct.lat = gps.location.lat();
        gpsStruct.lng = gps.location.lng();
      }
      if (gps.date.isValid())
      {
        gpsStruct.date = gps.date.value();
      }
      if (gps.time.isValid())
      {
        gpsStruct.time = gps.time.value();
      }
      if (gps.altitude.isValid())
      {
        gpsStruct.altitude = gps.altitude.meters();
      }
      if (gps.speed.isValid())
      {
        gpsStruct.speed = gps.speed.mps();
      }
      if (gps.course.isValid())
      {
        gpsStruct.course = gps.course.deg();
      }
      gpsStruct.systemTime = millis();

      xQueueSend(GPSQueue, &gpsStruct, 0);
      xQueueSend(DisplayGPSQueue, &gpsStruct, 0);
      xQueueSend(RadioGPSQueue, &gpsStruct, 0);
    }
    delay(100);
  }
}

void setup()
{

  Serial.begin(9600);

  Wire.begin(21, 22);

  pinMode(19, INPUT_PULLUP);
  spi.begin(18, 19, 23, SDCARDCS);

  if (!SD.begin(SDCARDCS, spi))
  {
    Serial.println("initialization failed!\n");
    return;
  }

  // Open files
  xSemaphoreTake(SDWriteSemaphore, portMAX_DELAY);
  pressureFile = SD.open("/pressure.txt", FILE_WRITE);
  accelGyroFile = SD.open("/accelGyro.txt", FILE_WRITE);
  gpsFile = SD.open("/gps.txt", FILE_WRITE);
  xSemaphoreGive(SDWriteSemaphore);

  xTaskCreatePinnedToCore(AccelAndGyro, "Accel", 40000, NULL, 1, &AccelAndGyroTask, 0);
  xTaskCreatePinnedToCore(Pressure, "pressure", 20000, NULL, 1, &PressureTask, 0);
  // xTaskCreatePinnedToCore(Display, "Display", 20000, NULL, 3, &DisplayTask, 0);
  xTaskCreatePinnedToCore(GPS, "GPS", 20000, NULL, 1, &GPSTask, 1);
  // xTaskCreatePinnedToCore(Flusher, "Flusher", 20000, &files, 1, &FlusherTask, 0);
}
int counter = 0;
void loop()
{

  if (uxQueueMessagesWaiting(AccelQueue) > 0)
  {
    AccelerationAndGyroStruct accelAndGyro;

    xQueueReceive(AccelQueue, &accelAndGyro, portMAX_DELAY);

    int returnSize = snprintf(NULL, 0, "%f,%f,%f,%f,%f,%f,%lu\n", accelAndGyro.ax, accelAndGyro.ay, accelAndGyro.az, accelAndGyro.gx, accelAndGyro.gy, accelAndGyro.gz, accelAndGyro.time);

    char *buffer = (char *)malloc(returnSize + 1);

    snprintf(buffer, returnSize + 1, "%f,%f,%f,%f,%f,%f,%lu\n", accelAndGyro.ax, accelAndGyro.ay, accelAndGyro.az, accelAndGyro.gx, accelAndGyro.gy, accelAndGyro.gz, accelAndGyro.time);

    accelGyroFile.print(buffer);
    counter++;
    free(buffer);
  }
  if (uxQueueMessagesWaiting(PressureQueue) > 0)
  {
    PressureStuct pressure;

    xQueueReceive(PressureQueue, &pressure, portMAX_DELAY);

    int returnSize = snprintf(NULL, 0, "%f,%f,%f,%lu\n", pressure.pressure, pressure.temperature, pressure.altitude, pressure.time);

    char *buffer = (char *)malloc(returnSize + 1);

    snprintf(buffer, returnSize + 1, "%f,%f,%f,%lu\n", pressure.pressure, pressure.temperature, pressure.altitude, pressure.time);

    pressureFile.print(buffer);
    counter++;
    free(buffer);
  }
  if (uxQueueMessagesWaiting(GPSQueue) > 0)
  {
    GPSStruct gps;

    xQueueReceive(GPSQueue, &gps, portMAX_DELAY);

    int returnSize = snprintf(NULL, 0, "%f,%f,%f,%f,%f,%lu,%lu,%lu\n", gps.lat, gps.lng, gps.altitude, gps.speed, gps.course, gps.date, gps.time, gps.systemTime);

    char *buffer = (char *)malloc(returnSize + 1);

    snprintf(buffer, returnSize + 1, "%f,%f,%f,%f,%f,%lu,%lu,%lu\n", gps.lat, gps.lng, gps.altitude, gps.speed, gps.course, gps.date, gps.time, gps.systemTime);

    gpsFile.print(buffer);
    counter++;
    free(buffer);
  }
  if (counter > 10)
  {
    counter = 0;
    accelGyroFile.flush();
    pressureFile.flush();
  }
  delay(1);
}
