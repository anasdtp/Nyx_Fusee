#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_NeoPixel.h>
#include <TeensyThreads.h>
#include <SD.h>
#include <Nyx.h>

//Les fonctions les plus critiques et où il y aura forcément du taf à faire : 
// 1 - void ManageInfo();
// 2 - void logDataToSD(uint32_t microseconds, Vector acc, Vector gyro, float temperature, uint8_t color[3], uint16_t compteurLigne);
// 3 - bool begin(TwoWire *wire, int pinDeclanchementParachute);
//  ET, last but not least, la liaison I2C, gérer dans le setup et dans fusee.begin
//  ²Wire2.setSDA(25);
//   Wire2.setSCL(24);
//   if(fusee.begin(&Wire2)){//Pin 24 et 25 pour l'accelerometre //24 25 ne fonctionne pas avec la teensy 3.1, PREND UNE TEENSY 4.1 ET C OKLM
//     //Serial.println("fusee begin success!");
//   }
//Il faut regarder quelle Wire utilisé pour les pins SDA SCL alternative

//IMPORTANT! -> Pour passer de la teensy 3.5 à la teensy 4.1 
//              il faut changer l'environnement platformio en choisisant,
//              pour la teensy 4.1 par ex, 'env:teensy41'
//              Il faut ensuite adapter la liaison I2C. Pour la teensy 4.1 il faut choisir Wire2 avec les pins de base (25, 24)
#define PIN_SDA 25
#define PIN_SCL 24

#define PIN1_WS2812B    4   // pin that connects to WS2812B
#define PIN2_WS2812B    5
#define PIN3_WS2812B    6
#define NUM_PIXELS      256  // The number of LEDs (pixels) on WS2812B

Adafruit_MPU6050 mpu;

Adafruit_NeoPixel matrix1(NUM_PIXELS, PIN1_WS2812B, NEO_GRB + NEO_KHZ800);//Mauvaise librairie, les couleurs sont a chiées
Adafruit_NeoPixel matrix2(NUM_PIXELS, PIN2_WS2812B, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel matrix3(NUM_PIXELS, PIN3_WS2812B, NEO_GRB + NEO_KHZ800);

Nyx fusee(&mpu, &matrix1, &matrix2, &matrix3, NUM_PIXELS);

void clignotementLedBuiltIn(uint32_t waitms);
void setup() {
  //Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  //Serial.println("Hello, world! Pret a tout Nixer!!");

  //Configuration des parametres de la fusee
  fusee.setTempsEchantionnageMPU(500);

  fusee.setSeuilGDeclenchementFusee(4);

  fusee.setTimeoutFlying(60 *1000 *1000);//1 minute

  fusee.setGetMpuInfoState(false);
  
  Wire2.setSDA(PIN_SDA);
  Wire2.setSCL(PIN_SCL);
  if(fusee.begin(&Wire2)){//Pin 24 et 25 pour l'accelerometre
    //Serial.println("fusee begin success!");
  }

  fusee.setGetMpuInfoState(true);//On lance la recolte des infos via liaison I2C

}

void loop() {
  static int etat = 0;
  switch (etat)
  {
  case 0:
    switch (fusee.rocketState)
    {
    case fusee.PRE_LAUNCH:
      clignotementLedBuiltIn(500);
      break;
    case fusee.ROCKET_IN_FLIGHT:
      clignotementLedBuiltIn(100);//Led clignote rapidement si lancement detecté
      break;
    default:
      clignotementLedBuiltIn(500);
      break;
    }
    
    if(fusee.ManageInfo()){
      //Alors fin de la mission, on est à terre
      //On arrete de recolter des infos de la MPU
      fusee.setGetMpuInfoState(false);
      //Serial.println("Fin de la mission");
      etat = 1;
    }
    
    break;
  case 1://Alors on clignote
    clignotementLedBuiltIn(250);
    fusee.clignotement(&matrix1, fusee.BLUE, fusee.WHITE,  100);
    fusee.clignotement(&matrix2, fusee.BLUE, fusee.WHITE,  100);
    fusee.clignotement(&matrix3, fusee.BLUE, fusee.WHITE,  100);
  break;
  
  default:
    break;
  }
}

void clignotementLedBuiltIn(uint32_t waitms){
  static bool etatLed = false;
  static uint32_t startTimeLED = millis();
  if((millis() - startTimeLED)>waitms){
    digitalWrite(LED_BUILTIN, etatLed);
    etatLed =!etatLed;
    startTimeLED = millis();
  }
}