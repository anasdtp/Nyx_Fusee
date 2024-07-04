#ifndef Nyx_h
#define Nyx_h
#include <Arduino.h>
#include <Geometry.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <TeensyThreads.h>

#include <SD.h>

typedef struct InfoNyx{
    uint32_t microseconds;
    reel accX, accY, accZ, gyroX, gyroY, gyroZ, temperature;
}InfoNyx;

#define SIZE_FIFO 2048
#define BUFFER_WRITE_SIZE 2048

class Nyx {
public:
    Nyx(Adafruit_MPU6050 *mpu, Adafruit_NeoPixel *matrix1, Adafruit_NeoPixel *matrix2, Adafruit_NeoPixel *matrix3, int numPixel);

    //I2C, Wire2 Pin 24 25 par defaut sur Teensy 4.1, 
    bool begin(TwoWire *wire, int pinDeclanchementParachute = 28);
    
    bool setupSD();
    void logDataToSD(uint32_t microseconds, Vector acc, Vector gyro, float temperature, uint8_t color[3], uint16_t compteurLigne);

    //Fonction la plus importante, qui gére la fusée à partir des infos
    bool ManageInfo();


    uint8_t GREEN[3] =  {0, 255, 0}, BLUE[3]    = {0, 0, 255}, 
            LIGHTBLUE[3] = {0, 255, 255}, YELLOW[3] = {255, 255, 0},
            RED[3]   =  {255, 0, 0}, WHITE[3]   = {255, 255, 255}, 
            BLACK[3] =  {0, 0, 0},   PURPLE[3]  = {238, 130, 238};
    void setLedColor(Adafruit_NeoPixel *matrix, uint8_t couleur[3]);
    void rainbow(Adafruit_NeoPixel *matrix, uint32_t waitms = 20, uint8_t brightness = 100);

    void clignotement(Adafruit_NeoPixel *matrix, uint8_t couleurON[3], uint8_t couleurOFF[3], uint32_t waitms = 750, uint8_t brightness = 255);

    enum RocketState {
        PRE_LAUNCH = 1,
        ROCKET_IN_FLIGHT = 2,
        ROCKET_LANDED = 3,
    };
    RocketState rocketState = PRE_LAUNCH;

    void setPinDeclanchementParachute(int pin){
        _pinDeclanchementParachute = pin;
        pinMode(_pinDeclanchementParachute, OUTPUT);
    }

    //Temps d'echantionnage en microseconds de la recolte de données de la mpu
    void setTempsEchantionnageMPU(uint32_t microseconds){
        _TempsEchantionnageMPU = microseconds;
    }

    //En us. //Une minute aprés le lancement, passer en mode post-vol
    void setTimeoutFlying(uint32_t microseconds){timeoutFlying = microseconds;}

    void setGetMpuInfoState(bool state){
        _flagGetMpuInfo = state;
    }
    
    //Valeur en nombre de g de la norme de l'acceleration qui signifie qu'on a decollé
    void setSeuilGDeclenchementFusee(reel G){
        _seuilLancementFusee = G* 9.81;
    }

    
    /**********************************************************************************/
    /*Fonctions suivante Non utilisées mais laissées pour une potentielle amélioration : */
    /**********************************************************************************/
    //Valeur en nombre de g de la norme de l'acceleration qui signifie qu'on vient de finir le vol
    void setSeuilGVolFiniFusee(reel G){
        _seuilFuseeVolFini = G* 9.81;
    }
    //Valeur en nombre de g de la norme de l'acceleration qui signifie qu'on chute
    void setSeuilGChuteFusee(reel G){
        _seuilFuseeEnChute = G* 9.81;
    }
    //Valeur en millisecond d'attente aprés la fin du vol pour déclencher le parachute
    void setWaitTimeLaunchParachute(uint32_t milliseconds){
        _waitTimeLaunchParachute = milliseconds;
    }
    /**********************************************************************************/


    

private:
    Adafruit_MPU6050 *_mpu;

    File _dataFile;
    
    InfoNyx _info[SIZE_FIFO]; uint16_t FIFO_Ecriture = 0;

    char dataBuffer[BUFFER_WRITE_SIZE];//Buffer d'ecriture dans la sd
    size_t bufferIndex = 0;
    
    int _NUM_PIXELS;
    
    Adafruit_NeoPixel *_matrix1;
    Adafruit_NeoPixel *_matrix2;
    Adafruit_NeoPixel *_matrix3;
    uint8_t couleurActuelle[3];

    int _pinDeclanchementParachute;


    reel _seuilLancementFusee;//Valeur de l'acceleration qui signifie qu'on a decollé
    reel _seuilFuseeVolFini;//Valeur de l'acceleration qui signifie qu'on vient de finir le vol
    reel _seuilFuseeEnChute;//Valeur de l'acceleration qui signifie qu'on chute
    uint32_t _startTimeLaunchParachute, _waitTimeLaunchParachute;//ms, Attente avant de lancer le parachute
    uint32_t _startTimeUsFlying; //us, Debut du vol en us

    uint32_t _TempsEchantionnageMPU;//Temps d'echantionnage en microseconds recolte données de la mpu

    uint32_t timeoutFlying = 60 *1000 *1000;//Une minute aprés le lancement, passer en mode post-vol


    int readCounter();
    void writeCounter(int counter);
    
    //Fonction en parralléle qui recolte les données de la mpu
    bool _flagGetMpuInfo = false;
    void threadMpuInfo();
    void getMpuInfo();
    static void threadMpuInfoWrapper(void* arg) {
        Nyx* nyxInstance = static_cast<Nyx*>(arg);
        nyxInstance->threadMpuInfo();
    }
    

    
};

#endif //Nyx_h