#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_NeoPixel.h>
#include <TeensyThreads.h>
#include <SD.h>
#include <Nyx.h>

#define PIN1_WS2812B    4   // pin that connects to WS2812B
#define PIN2_WS2812B    5
#define PIN3_WS2812B    6
#define NUM_PIXELS      256  // The number of LEDs (pixels) on WS2812B

Adafruit_MPU6050 mpu;

Adafruit_NeoPixel matrix1(NUM_PIXELS, PIN1_WS2812B, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel matrix2(NUM_PIXELS, PIN2_WS2812B, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel matrix3(NUM_PIXELS, PIN3_WS2812B, NEO_GRB + NEO_KHZ800);

Nyx fusee(&mpu, &matrix1, &matrix2, &matrix3, NUM_PIXELS);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Hello, world! Pret a tout Nixer!!");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Wire1.setSDA(25);
  Wire1.setSCL(24);
  fusee.begin(&Wire1);//Pin 24 et 25 pour l'accelerometre

  //Configuration des parametres de la fusee
  fusee.setTempsEchantionnageMPU(50);

  fusee.setSeuilGDeclenchementFusee(4);
  fusee.setSeuilGVolFiniFusee(1.5);
  fusee.setSeuilGChuteFusee(0.5);

  fusee.setWaitTimeLaunchParachute(1000);//Une seconde

}

void loop() {
  fusee.loop();
}