#include "Nyx.h"

Nyx::Nyx(Adafruit_MPU6050 *mpu, Adafruit_NeoPixel *matrix1, Adafruit_NeoPixel *matrix2, Adafruit_NeoPixel *matrix3, int numPixel){
    _mpu = mpu;

    _matrix1 = matrix1;
    _matrix2 = matrix2;
    _matrix3 = matrix3;
    _NUM_PIXELS = numPixel;
    rocketState = PRE_LAUNCH;
    setTempsEchantionnageMPU(50);
    setSeuilGDeclenchementFusee(5);//5G en m/s^2
    setSeuilGVolFiniFusee(1.5);
    setSeuilGChuteFusee(0.5);
    setWaitTimeLaunchParachute(1000);
}

bool Nyx::begin(TwoWire *wire, int pinDeclanchementParachute){
    _pinDeclanchementParachute = pinDeclanchementParachute;
    pinMode(_pinDeclanchementParachute, OUTPUT);
    digitalWrite(_pinDeclanchementParachute, LOW);

    _matrix1->begin();
    _matrix2->begin();
    _matrix3->begin();
    
    setLedColor(_matrix1, RED);
    setLedColor(_matrix2, RED);
    setLedColor(_matrix3, RED);
    
    setupSD();

    // while(!_mpu->begin(MPU6050_I2CADDR_DEFAULT, wire)){
    //     Serial.println("Failed to find MPU6050 chip...");
    //     delay(500);
    // }
    if(!_mpu->begin(MPU6050_I2CADDR_DEFAULT, wire)){
        Serial.println("Failed to find MPU6050 chip...");
        return false;
    }
    Serial.println("MPU6050 Found!");

    _mpu->setAccelerometerRange(MPU6050_RANGE_16_G);//Acceleration maximal
    // mpu.getAccelerometerRange()
    Serial.println("+-16G");

    _mpu->setGyroRange(MPU6050_RANGE_250_DEG);

    _mpu->setFilterBandwidth(MPU6050_BAND_21_HZ);

    setGetMpuInfoState();
    threads.addThread(Nyx::threadMpuInfoWrapper, this);
    

    return true;
}

void Nyx::setupSD() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Erreur lors de l'initialisation de la carte SD!");
        return;
    }
    _dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (!_dataFile) {
        Serial.println("Erreur lors de l'ouverture du fichier sur la carte SD!");
    }
}

void Nyx::logDataToSD(uint32_t microseconds, Vector acc, Vector gyro, float temperature, uint8_t color[3]) {
    if (_dataFile) {
        _dataFile.print(microseconds);
        _dataFile.print(", ");
        _dataFile.print(acc.x);
        _dataFile.print(", ");
        _dataFile.print(acc.y);
        _dataFile.print(", ");
        _dataFile.print(acc.z);
        _dataFile.print(", ");
        _dataFile.print(gyro.x);
        _dataFile.print(", ");
        _dataFile.print(gyro.y);
        _dataFile.print(", ");
        _dataFile.print(gyro.z);
        _dataFile.print(", ");
        _dataFile.print(temperature);
        _dataFile.print(", ");
        _dataFile.print(color[0]);
        _dataFile.print(", ");
        _dataFile.print(color[1]);
        _dataFile.print(", ");
        _dataFile.print(color[2]);
        _dataFile.println();
        _dataFile.flush();
    } else {
        Serial.println("Erreur d'écriture sur la carte SD!");
    }
}

void Nyx::loop(){
    // getMpuInfo();//->Doit etre dans une thread

    ManageInfo();
}

void Nyx::threadMpuInfo(){
    setGetMpuInfoState();
    while(true){
        getMpuInfo();
        threads.delay_us(_TempsEchantionnageMPU);//Ce n'est pas un delay bloquant
    }
}

void Nyx::getMpuInfo(){
    static uint32_t startTimeVoie = 0, firstTime = micros();//en microseconds 
    static sensors_event_t a, g, temp;

    if(!_flagGetMpuInfo){return;}

    if((micros() - startTimeVoie)>_TempsEchantionnageMPU){
        startTimeVoie = micros();
        _mpu->getEvent(&a, &g, &temp);

        // Serial.println("getMpuInfo()");
        _info[FIFO_Ecriture].microseconds = startTimeVoie - firstTime;//dt
        
        _info[FIFO_Ecriture].accX = a.acceleration.x;
        _info[FIFO_Ecriture].accY = a.acceleration.y;
        _info[FIFO_Ecriture].accZ = a.acceleration.z;

        _info[FIFO_Ecriture].gyroX = g.gyro.x;
        _info[FIFO_Ecriture].gyroY = g.gyro.y;
        _info[FIFO_Ecriture].gyroZ = g.gyro.z;

        _info[FIFO_Ecriture].temperature = temp.temperature;

        FIFO_Ecriture = (FIFO_Ecriture + 1) % SIZE_FIFO;//On rajoute à la pile l'info et on incremente le curseur
    }
}

void Nyx::ManageInfo(){
    static uint16_t FIFO_lecture = 0, FIFO_occupation = 0, FIFO_max_occupation = 0;
    static uint32_t startTimeFlying = 0, timeoutFlying = 8000;//Sécurité si il y a un probléme avec la MPU pour declencher le parachuter aprés 8 secondes de vol
    static uint32_t startTimeFalling = 0, waitTimeAfterFalling = 1000*60;//Attendre 1 minutes et on peut dire que la fusée est par terre

    FIFO_occupation = FIFO_Ecriture - FIFO_lecture;
    if(FIFO_occupation<0){FIFO_occupation=FIFO_occupation+SIZE_FIFO;}
    if(FIFO_max_occupation<FIFO_occupation){FIFO_max_occupation=FIFO_occupation;}
    if(!FIFO_occupation){return;}

    Vector acc(_info[FIFO_lecture].accX,_info[FIFO_lecture].accY,_info[FIFO_lecture].accZ);
    reel acc_norm = acc.norm();
    Vector gyro(_info[FIFO_lecture].gyroX,_info[FIFO_lecture].gyroY,_info[FIFO_lecture].gyroZ);

    /****************print pour debug****************/
    //A mettre en commentaire aprés
    Serial.printf("FIFO_lecture : %d, FIFO_Ecriture : %d\n", FIFO_lecture, FIFO_Ecriture);
    Serial.printf("dt us : %d\n", _info[FIFO_lecture].microseconds);
    Serial.print("Acceleration X: ");
    Serial.print(acc.x);
    Serial.print(", Y: ");
    Serial.print(acc.y);
    Serial.print(", Z: ");
    Serial.print(acc.z);
    Serial.println(" m/s^2");

    Serial.print("Acceleration norm : ");
    Serial.println(acc_norm);

    Serial.print("Rotation X: ");
    Serial.print(gyro.x);
    Serial.print(", Y: ");
    Serial.print(gyro.y);
    Serial.print(", Z: ");
    Serial.print(gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(_info[FIFO_lecture].temperature);
    Serial.println(" degC");

    // Serial.printf("Couleur actuelle : R : %d, G : %d, B : %d\n", _info[FIFO_lecture].color[0], _info[FIFO_lecture].color[1], _info[FIFO_lecture].color[2]);

    Serial.println("");
    /************************************************/

    switch (rocketState)
    {
    case PRE_LAUNCH:{
        rainbow(_matrix1);
        rainbow(_matrix2);
        rainbow(_matrix3);
        if (acc_norm > _seuilLancementFusee) { // 5G par ex en m/s^2
            Serial.println("Demarrage de la fusee détectee!*******************");
            rocketState = ROCKET_IN_FLIGHT;
            startTimeFlying = millis();
            // Initialiser l'enregistrement des données ici
        }
    }break;
    case ROCKET_IN_FLIGHT:{
        //Calculer la couleur a avoir par rapport à la norme de l'acceleration
        reel factor = (acc_norm - _seuilFuseeVolFini) / (_seuilLancementFusee - _seuilFuseeVolFini);
        factor = constrain(factor, 0.0, 1.0); // S'assurer que le facteur est entre 0 et 1
        for (int i = 0; i < 3; i++) {
            couleurActuelle[i] = BLUE[i] + factor * (RED[i] - BLUE[i]);//Pour passer du bleu au rouge par rapport à la norme de l'acceleration
        }

        // Enregistrer les données dans la carte SD
        logDataToSD(_info[FIFO_lecture].microseconds, acc, gyro, _info[FIFO_lecture].temperature, couleurActuelle);

        setLedColor(_matrix1, couleurActuelle);
        setLedColor(_matrix2, couleurActuelle);
        setLedColor(_matrix3, couleurActuelle);

        // Vérifier si l'accélération chute pour détecter la fin du vol
        if ((acc_norm < _seuilFuseeVolFini) || ((millis() - startTimeFlying)>=timeoutFlying)) { // Seuil pour détecter la fin du vol
            Serial.println("Fin de la phase vol détectee! Largage du parachute");
            rocketState = ROCKET_LAUNCH_PARACHUTE;

            setLedColor(_matrix1, WHITE);
            setLedColor(_matrix2, WHITE);
            setLedColor(_matrix3, WHITE);
            // Déclencher le parachute
            _startTimeLaunchParachute = millis();
        }
    }break;
    case ROCKET_LAUNCH_PARACHUTE:{
        if((millis() - _startTimeLaunchParachute) >= _waitTimeLaunchParachute){
            digitalWrite(_pinDeclanchementParachute, HIGH); //Lancer le parachute
            if (acc_norm < _seuilFuseeEnChute) { // Seuil pour détecter la chute
                rocketState = ROCKET_FALLING;
                Serial.println("En chute..");
            }
            startTimeFalling = millis();
        }
    }break;
    case ROCKET_FALLING:{
        logDataToSD(_info[FIFO_lecture].microseconds, acc, gyro, _info[FIFO_lecture].temperature, couleurActuelle);
        // Attendre 1 minutes et on peut dire que la fusée est par terre
        if((millis() - startTimeFalling)>= waitTimeAfterFalling){
            rocketState = ROCKET_LANDED;
            Serial.println("Fusee est par terre!");
            digitalWrite(_pinDeclanchementParachute, LOW);
        }

        
    }break;
    case ROCKET_LANDED:{
        
        clignotement(_matrix1, BLUE, WHITE);
        clignotement(_matrix2, BLUE, WHITE);
        clignotement(_matrix3, BLUE, WHITE);

    }break;
    default:
        break;
    }

    // delay(100);

    FIFO_lecture = (FIFO_lecture + 1) % SIZE_FIFO;
}


void Nyx::setLedColor(Adafruit_NeoPixel *matrix, uint8_t couleur[3]){
    matrix->clear();
    for (int i = 0; i < _NUM_PIXELS; i++)
    {
        matrix->setPixelColor(i, matrix->Color(couleur[0], couleur[1], couleur[2]));
    }
    matrix->show();

    couleurActuelle[0] = couleur[0];
    couleurActuelle[1] = couleur[1];
    couleurActuelle[2] = couleur[2];
}

void Nyx::rainbow(Adafruit_NeoPixel *matrix, uint32_t waitms, uint8_t brightness) {
  static uint32_t start_time = 0; static long firstPixelHue = 0;
  if((millis()-start_time)>waitms){
    // Serial.println("yes");
      start_time = millis();
      matrix->rainbow(firstPixelHue, -1, 255, brightness);
      matrix->show();
      firstPixelHue = (firstPixelHue + 256)%(3*65536);
    //   vTaskDelay(pdMS_TO_TICKS(waitms));
  }
}

void Nyx::clignotement(Adafruit_NeoPixel *matrix, uint8_t couleurON[3], uint8_t couleurOFF[3], uint32_t waitms, uint8_t brightness) {
  static uint32_t start_time = 0; static bool colorChoice = true;
  if((millis()-start_time)>waitms){
    // Serial.println("yes");
      start_time = millis();
      setLedColor(matrix, (colorChoice?couleurON:couleurOFF));
      colorChoice = !colorChoice;
  }
}