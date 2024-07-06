#include "Nyx.h"

Nyx::Nyx(Adafruit_MPU6050 *mpu, Adafruit_NeoPixel *matrix1, Adafruit_NeoPixel *matrix2, Adafruit_NeoPixel *matrix3, int numPixel){
    _mpu = mpu;

    _matrix1 = matrix1;
    _matrix2 = matrix2;
    _matrix3 = matrix3;
    _NUM_PIXELS = numPixel;
    FIFO_Ecriture = 0;
    setGetMpuInfoState(false);
    rocketState = PRE_LAUNCH;
    setTempsEchantionnageMPU(1000);
    setSeuilGDeclenchementFusee(5);//5G en m/s^2
    setSeuilGVolFiniFusee(1.5);//Sert a rien 
    setSeuilGChuteFusee(0.5);//Sert a rien 
    setTimeoutFlying(60 *1000 *1000);
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

    rocketState = PRE_LAUNCH;

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

    setGetMpuInfoState(false);
    FIFO_Ecriture = 0;
    threads.addThread(Nyx::threadMpuInfoWrapper, this);
    
    return setupSD();
}

// Fonction pour lire le compteur depuis le fichier compteur.txt
int Nyx::readCounter() {
    File counterFile = SD.open("Nyx/compteur.txt");
    int counter = 0;
    if (counterFile) {
        counter = counterFile.parseInt();
        counterFile.close();
    }
    return counter;
}

// Fonction pour écrire le compteur dans le fichier compteur.txt
void Nyx::writeCounter(int counter) {
    File counterFile = SD.open("Nyx/compteur.txt", FILE_WRITE);
    if (counterFile) {
        counterFile.seek(0);  // Aller au début du fichier
        counterFile.print(counter);
        counterFile.close();
    }
}

bool Nyx::setupSD() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("Erreur lors de l'initialisation de la carte SD!");
        return false;
    }
    int compteur = readCounter();
    String randomFilePath = "Nyx/datalog" + String(compteur) + ".txt";
    _dataFile = SD.open(randomFilePath.c_str(), FILE_WRITE);
    if (!_dataFile) {
        Serial.println("Erreur lors de l'ouverture du fichier sur la carte SD!");
    }

    // Mettre à jour le compteur et l'écrire dans compteur.txt
    compteur++;
    writeCounter(compteur);
    return true;
}

void Nyx::logDataToSD(uint32_t microseconds, Vector acc, Vector gyro, float temperature, uint8_t color[3], uint16_t compteurLigne) {
    if (_dataFile) {
        // _dataFile.print(microseconds);
        // _dataFile.print(",");
        // _dataFile.print(acc.x);
        // _dataFile.print(",");
        // _dataFile.print(acc.y);
        // _dataFile.print(",");
        // _dataFile.print(acc.z);
        // _dataFile.print(",");
        // _dataFile.print(gyro.x);
        // _dataFile.print(",");
        // _dataFile.print(gyro.y);
        // _dataFile.print(",");
        // _dataFile.print(gyro.z);
        // _dataFile.print(",");
        // _dataFile.print(temperature);
        // _dataFile.print(",");
        // _dataFile.print(color[0]);
        // _dataFile.print(",");
        // _dataFile.print(color[1]);
        // _dataFile.print(",");
        // _dataFile.print(color[2]);
        // _dataFile.print(",");
        // _dataFile.print(compteurLigne);
        // _dataFile.println();
        // _dataFile.flush();

        //Façon plus efficace et plus rapide que ci-dessus:
        // 'dataBuffer + bufferIndex' indique où commencer à écrire les nouvelles données
        int len = snprintf(dataBuffer + bufferIndex, BUFFER_WRITE_SIZE - bufferIndex,
                           "%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d\n",
                           microseconds, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z,
                           temperature, color[0], color[1], color[2], compteurLigne);
        
        bufferIndex += len;

        // Si le tampon est plein, écrire les données sur la carte SD
        if (bufferIndex >= BUFFER_WRITE_SIZE - 50) { // 50 pour éviter les débordements
            _dataFile.write(dataBuffer, bufferIndex);
            _dataFile.flush();
            bufferIndex = 0; // Réinitialiser l'index du tampon
            // Serial.println("SD");
        }
        

    } else {
        Serial.println("Erreur d'ecriture sur la carte SD!");
    }
}

void Nyx::threadMpuInfo(){
    FIFO_Ecriture = 0;
    while(true){
        if(_flagGetMpuInfo){getMpuInfo();}
        threads.delay_us(50);//Ce n'est pas un delay bloquant
    }
}

void Nyx::getMpuInfo(){
    static uint32_t startTimeVoie = 0;//en microseconds 
    static sensors_event_t a, g, temp; static bool initFirstTime = true;

    if((micros() - startTimeVoie)>_TempsEchantionnageMPU){
        if(initFirstTime){initFirstTime = false; _startTimeUsFlying = micros();}
        startTimeVoie = micros();
        _mpu->getEvent(&a, &g, &temp);

        // Serial.println("getMpuInfo()");
        _info[FIFO_Ecriture].microseconds = startTimeVoie - _startTimeUsFlying;//dt
        
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

bool Nyx::ManageInfo(){
    static int32_t FIFO_lecture = 0, FIFO_occupation = 0, FIFO_max_occupation = 0;
    // static uint32_t startTimeFalling = 0, waitTimeAfterFalling = 1000*60;//Attendre 1 minutes et on peut dire que la fusée est par terre
    static const reel AccelMax = 12. * 9.81;
    FIFO_Ecriture = 500;
    bool output = false;//Si jamais output est à true, on a terminé la mission et on est au sol, donc faire clignoter les matrices led

    FIFO_occupation = FIFO_Ecriture - FIFO_lecture;
    if(FIFO_occupation<0){FIFO_occupation=FIFO_occupation+SIZE_FIFO;}
    if(FIFO_max_occupation<FIFO_occupation){FIFO_max_occupation=FIFO_occupation;}
    if(!FIFO_occupation){return output;}

    Vector acc(_info[FIFO_lecture].accX,_info[FIFO_lecture].accY,_info[FIFO_lecture].accZ);
    reel acc_norm = acc.norm();
    Vector gyro(_info[FIFO_lecture].gyroX,_info[FIFO_lecture].gyroY,_info[FIFO_lecture].gyroZ);

    /****************print pour debug****************/
    //A mettre en commentaire aprés
    /*Serial.printf("FIFO_lecture : %d, FIFO_Ecriture : %d\n", FIFO_lecture, FIFO_Ecriture);
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

    Serial.println("");//*/
    /************************************************/

    switch (rocketState)
    {
    case PRE_LAUNCH:{
        // rainbow(_matrix1);
        // rainbow(_matrix2);
        // rainbow(_matrix3);
        clignotement(_matrix1, LIGHTBLUE, YELLOW, 100);
        clignotement(_matrix2, LIGHTBLUE, YELLOW, 100);
        clignotement(_matrix3, LIGHTBLUE, YELLOW, 100);

        if (acc_norm > _seuilLancementFusee) { // 5G par ex en m/s^2, ALORS LANCEMENT DE LA FUSEE
            Serial.println("Demarrage de la fusee détectee!*******************");
            rocketState = ROCKET_IN_FLIGHT;
            digitalWrite(_pinDeclanchementParachute, HIGH); //Lancer le compteur du parachute
            _startTimeUsFlying = micros();
            // Initialiser l'enregistrement des données ici
        }
    }break;
    case ROCKET_IN_FLIGHT:{
        //Calculer la couleur a avoir par rapport à la norme de l'acceleration
        static reel factor;
        factor = acc_norm /AccelMax;
        factor = constrain(factor, 0.0, 1.0); // S'assurer que le facteur est entre 0 et 1
        for (int i = 0; i < 3; i++) {
            couleurActuelle[i] = BLUE[i] + factor * (RED[i] - BLUE[i]);//Pour passer du bleu au rouge par rapport à la norme de l'acceleration
        }

        // Enregistrer les données dans la carte SD
        logDataToSD(_info[FIFO_lecture].microseconds, acc, gyro, _info[FIFO_lecture].temperature, couleurActuelle, FIFO_lecture);

        setLedColor(_matrix1, couleurActuelle);
        setLedColor(_matrix2, couleurActuelle);
        setLedColor(_matrix3, couleurActuelle);

        // Vérifier si l'accélération chute pour détecter la fin du vol
        if ((micros() - _startTimeUsFlying)>=timeoutFlying) { // Seuil pour détecter la fin du vol
            Serial.println("Fin de la phase vol.");
            rocketState = ROCKET_LANDED;
            setGetMpuInfoState(false);//Arreter d'enregristrer les données de la mpu
            if(abs(FIFO_occupation)<=2){
                output = true;
            }
            
            setLedColor(_matrix1, WHITE);
            setLedColor(_matrix2, WHITE);
            setLedColor(_matrix3, WHITE);
            // Déclencher le parachute
            _startTimeLaunchParachute = millis();
        }
    }break;
    case ROCKET_LANDED:{
        Serial.println("ROCKET_LANDED");
        logDataToSD(_info[FIFO_lecture].microseconds, acc, gyro, _info[FIFO_lecture].temperature, couleurActuelle, FIFO_lecture);

        clignotement(_matrix1, BLUE, WHITE);
        clignotement(_matrix2, BLUE, WHITE);
        clignotement(_matrix3, BLUE, WHITE);

        if(abs(FIFO_occupation)<=2){
            output = true;     
        }
    }break;
    default:
        break;
    }

    FIFO_lecture = (FIFO_lecture + 1) % SIZE_FIFO;
    return output;
}


void Nyx::setLedColor(Adafruit_NeoPixel *matrix, uint8_t couleur[3]){
    matrix->clear();
    for (int i = 0; i < _NUM_PIXELS; i++)
    {
        matrix->setPixelColor(i, matrix->Color(couleur[1], couleur[0], couleur[2]));
    }
    matrix->show();

    couleurActuelle[0] = couleur[0];
    couleurActuelle[1] = couleur[1];
    couleurActuelle[2] = couleur[2];
}

void Nyx::rainbow(Adafruit_NeoPixel *matrix, uint32_t waitms, uint8_t brightness) {//Les leds ne sont pas les bonnes, il faut changer la librarie
  static uint32_t start_time = 0; static long firstPixelHue = 0;
  if((millis()-start_time)>waitms){
    Serial.println("yes");
      start_time = millis();
      matrix->rainbow(firstPixelHue, 1, 250, brightness);
      matrix->show();
      firstPixelHue = (firstPixelHue + 256)%(3*65536);
    //   vTaskDelay(pdMS_TO_TICKS(waitms));
  }
}

void Nyx::clignotement(Adafruit_NeoPixel *matrix, uint8_t couleurON[3], uint8_t couleurOFF[3], uint32_t waitms, uint8_t brightness) {
  static uint32_t start_time = 0; static bool colorChoice = true; static uint8_t color[3];
  if((millis()-start_time)>waitms){
        // Serial.println("yes");
        start_time = millis();
        if(colorChoice){
            color[0] = map(couleurON[0], 0, 255, 0, brightness); 
            color[1] = map(couleurON[1], 0, 255, 0, brightness);
            color[2] = map(couleurON[2], 0, 255, 0, brightness);
        }
        else{
            color[0] = map(couleurOFF[0], 0, 255, 0, brightness); 
            color[1] = map(couleurOFF[1], 0, 255, 0, brightness);
            color[2] = map(couleurOFF[2], 0, 255, 0, brightness);
        }  
        setLedColor(matrix, color);
        colorChoice = !colorChoice;
  }
}