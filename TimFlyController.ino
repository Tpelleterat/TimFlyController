/*
   TimFly version 1

   Serial communications :
   Input :
    -> "i" : Initialisation. A envoyer au premier démarrage pour s'assurer que la communication est bien établie.
    -> "t" : Décollage. A envoyer en deuxième pour indiquer que l'on souhaite décoller
*/
#include <Servo.h>

//const int INITIALISATIONLEDPIN = 13;      // Pin de la led pour voir si il y a eu initialisation
const int ACCELERMETER_X_PIN = 17;//0;         // Pin de l'axe X de l'accéléromètre - 0
const int ACCELERMETER_Y_PIN = 18;//1;         // Pin de l'axe Y de l'accéléromètre - 1
const char SERIAL_ACTION_ELEVATION = 'E'; // Lettre de l'action de changement d'haltitude reçu
const char SERIAL_ACTION_PICH = 'P';      // Lettre de l'action de changement du tangage reçu
const char SERIAL_ACTION_ROLL = 'R';      // Lettre de l'action de changement roulis reçu
const char SERIAL_ACTION_NEGATIF = 'N';   // Lettre de l'action pour indiquer une valeur négative
const char SERIAL_ACTION_STOP = 'S';      // Lettre de l'action pour arrêter le drone
const int ESC_FRONT_LEFT_PIN = 6;//3;         // Pin de l'ESC avant gauche
const int ESC_FRONT_RIGHT_PIN = 10;//5;        // Pin de l'ESC avant droite
const int ESC_BACK_LEFT_PIN = 5;//6;          // Pin de l'ESC arrière gauche
const int ESC_BACK_RIGHT_PIN = 9;//9;         // Pin de l'ESC arrière droite
const char SERIAL_ORDER_INIT_OK = 'i';    // Lettre l'orde d'inisialisation ok

int initialized = 0;            // Permet de savoir si la communication a été fait avec le cerveau
char action;                    // Serial communication : Action en cours
int isNegatif = 0;              // Serial communication : Indique si la valeur est négative
char value[5];                  // Serial communication : Permet de stocker les chiffres de l'action pour ça construction
int indexAction = 0;            // Serial communication : Index permettant de parcourir le tableau de la valeur pour le remplir

char motorCalibrationPositionLetter;              // Serial communication : Selection de la position du moteur à calibrer 'F' pour front ou 'B' pour back
char motorCalibrationLateralLetter;               // Serial communication : Selection de la position du moteur à calibrer 'L' pour left ou 'R' pour right
char motorCalibrationSerialData[5];               // Serial communication : Permet de stocker les chiffres de l'action pour ça construction
int motorCalibrationSerialIndex = 0;              // Serial communication : Index permettant de parcourir le tableau de la valeur pour le remplir
const char SERIAL_MOTORCALIBRATION_FRONT = 'F';   // Lettre de position Front pour le calibrage des moteurs
const char SERIAL_MOTORCALIBRATION_BACK = 'B';    // Lettre de position Back pour le calibrage des moteurs
const char SERIAL_MOTORCALIBRATION_LEFT = 'L';    // Lettre de position Left pour le calibrage des moteurs
const char SERIAL_MOTORCALIBRATION_RIGHT = 'R';   // Lettre de position Right pour le calibrage des moteurs
int FrontLeftMotorCalibrageIndice = 0;          // Indice de calibration pour la valeur minimum du moteur avant gauche
int FrontRightMotorCalibrageIndice = 0;        // Indice de calibration pour la valeur minimum du moteur avant droite
int BackLeftMotorCalibrageIndice = 0;         // Indice de calibration pour la valeur minimum du moteur arrière gauche
int BackRightMotorCalibrageIndice = 0;        // Indice de calibration pour la valeur minimum du moteur arrière droite

int ElevationIndice = 0; // Indice de hauteur. Permet de gérer la puissance des moteurs pour l'altitude
int PichIndice = 0;      // Indice de tangage. Permet d'incliner le drone vers l'avant ou l'arrière pour avancer ou reculer
long ResetPichTimer;     // Variable permettant de stocker le timestamp pour la reinitialisation du pich
int RollIndice = 0;      // Indice de roulis. Permet d'incliner le drone vers la gauche ou la droite pour le faire faire un virage
long ResetRollTimer;     // Variable permettant de stocker le timestamp pour la reinitialisation du roll

int NextFrontLeftMotorValue = -1;       // Valeur précédente du moteur avant gauche
int NextFrontRightMotoValue = -1;       // Valeur précédente du moteur avant droite
int NextBackLeftMotorValue = -1;        // Valeur précédente du moteur arrière gauche
int NextBackRightMotoValue = -1;        // Valeur précédente du moteur arrière droite

int FrontLeftMotorStabilisationIndice = 0;    // Indice de stabilisation du moteur avant gauche. Permet de changer la puissance du moteur pour retrouver une stabilité
int FrontRightMotorStabilisationIndice = 0;    // Indice de stabilisation du moteur avant droite. Permet de changer la puissance du moteur pour retrouver une stabilité
int BackLeftMotorStabilisationIndice = 0;     // Indice de stabilisation du moteur arrière gauche. Permet de changer la puissance du moteur pour retrouver une stabilité
int BackRightMotorStabilisationIndice = 0;     // Indice de stabilisation du moteur arrière gauche. Permet de changer la puissance du moteur pour retrouver une stabilité

int FrontLeftMotorMovementIndice = 0;     // Indice de mouvement du moteur avant gauche. Permet de changer la puissance du moteur pour le déplacement
int FrontRightMotoMovementIndice = 0;     // Indice de mouvement du moteur avant droite. Permet de changer la puissance du moteur pour le déplacement
int BackLeftMotorMovementIndice = 0;      // Indice de mouvement du moteur arrière gauche. Permet de changer la puissance du moteur pour le déplacement
int BackRightMotoMovementIndice = 0;      // Indice de mouvement du moteur arrière gauche. Permet de changer la puissance du moteur pour le déplacement

Servo escFrontLeft;       // Servo de contrôle esc avant gauche
Servo escFrontRight;      // Servo de contrôle esc avant droite
Servo escBackLeft;        // Servo de contrôle esc avant gauche
Servo escBackRight;       // Servo de contrôle esc avant droite

char inData[20];    // Serial communication : Permet de stocker les données reçu de la communication série reçu pour comparaison
char inChar = -1;   // Serial communication : Permet de stocker un charactère de la communication série reçu pour comparaison
byte index = 0;     // Serial communication : Permet de naviger dans la variable de stockage de donnée pour comparaison

int XCalibrage = 0;   // Permet de calibrage de l'axe X pour démarrer à zéro
int YCalibrage = 0;   // Permet de calibrage de l'axe Y pour démarrer à zéro

int Stop = 0;
/***************************************************************************************/

void setup() {
  // Inisialisation communication serie
  Serial.begin(9600);

  escFrontLeft.attach(ESC_FRONT_LEFT_PIN);
  escFrontRight.attach(ESC_FRONT_RIGHT_PIN);
  escBackLeft.attach(ESC_BACK_LEFT_PIN);
  escBackRight.attach(ESC_BACK_RIGHT_PIN);

  FrontLeftMotorCalibrageIndice = 990;
  FrontRightMotorCalibrageIndice = 990;
  BackLeftMotorCalibrageIndice = 990;
  BackRightMotorCalibrageIndice = 990;

  setFrontLeftMotorValues(FrontLeftMotorCalibrageIndice);
  setFrontRightMotorValues(FrontRightMotorCalibrageIndice);
  setBackLeftMotorValues(BackLeftMotorCalibrageIndice);
  setBackRightMotorValues(BackRightMotorCalibrageIndice);
}

void loop() {
  if (Stop == 0) {
    if (initialized == 0) {
      calibrate();
      checkSerialInitialisation();
    } else {
      checkSerialMovements();
      manageStability();
      setMotorsValues();
    }
  } else {
    setFrontLeftMotorValues(0);
    setFrontRightMotorValues(0);
    setBackLeftMotorValues(0);
    setBackRightMotorValues(0);
  }
}

/* Initialisation **************************************************************************************/

/*
   Permet le lancement du calibrage
*/
void calibrate() {
  XCalibrage = getXAxe(0);
  YCalibrage = getYAxe(0);
}

/*
   Vérification dans la communication série si l'inisialisation est envoyée
*/
void checkSerialInitialisation() {
  while (Serial.available() > 0)
  {
    char storageActionChar = Serial.read();

    if (storageActionChar == SERIAL_ORDER_INIT_OK) {
      initialized = 1;
      Serial.print(":INIT_OK;");

      clearSerialMotorCalibrationData();
    }
    else if (storageActionChar == SERIAL_MOTORCALIBRATION_FRONT || storageActionChar == SERIAL_MOTORCALIBRATION_BACK) {
      motorCalibrationPositionLetter = storageActionChar;
    } else if (storageActionChar == SERIAL_MOTORCALIBRATION_LEFT || storageActionChar == SERIAL_MOTORCALIBRATION_RIGHT) {
      motorCalibrationLateralLetter = storageActionChar;
    } else if (storageActionChar == ';') {
      if (motorCalibrationPositionLetter != -1 && motorCalibrationLateralLetter != -1) {
        motorsCalibration();
      }

      clearSerialMotorCalibrationData();
    } else {
      motorCalibrationSerialData[motorCalibrationSerialIndex] = storageActionChar;
      motorCalibrationSerialIndex++;
      motorCalibrationSerialData[motorCalibrationSerialIndex] = '\0';
    }
  }
}

/*
   Applique la valeur de calibrage aux moteurs
*/
void motorsCalibration() {
  int motorCalibrationValue = atoi(motorCalibrationSerialData);

  if (motorCalibrationPositionLetter == SERIAL_MOTORCALIBRATION_FRONT && motorCalibrationLateralLetter == SERIAL_MOTORCALIBRATION_LEFT) {
    FrontLeftMotorCalibrageIndice = motorCalibrationValue;
    setFrontLeftMotorValues(FrontLeftMotorCalibrageIndice);
  } else if (motorCalibrationPositionLetter == SERIAL_MOTORCALIBRATION_FRONT && motorCalibrationLateralLetter == SERIAL_MOTORCALIBRATION_RIGHT) {
    FrontRightMotorCalibrageIndice = motorCalibrationValue;
    setFrontRightMotorValues(FrontRightMotorCalibrageIndice);
  } else if (motorCalibrationPositionLetter == SERIAL_MOTORCALIBRATION_BACK && motorCalibrationLateralLetter == SERIAL_MOTORCALIBRATION_LEFT) {
    BackLeftMotorCalibrageIndice = motorCalibrationValue;
    setBackLeftMotorValues(BackLeftMotorCalibrageIndice);
  } else if (motorCalibrationPositionLetter == SERIAL_MOTORCALIBRATION_BACK && motorCalibrationLateralLetter == SERIAL_MOTORCALIBRATION_RIGHT) {
    BackRightMotorCalibrageIndice = motorCalibrationValue;
    setBackRightMotorValues(BackRightMotorCalibrageIndice);
  }
}

/*
   Vide les valeurs de la communication serie du calibrage des moteurs
*/
void clearSerialMotorCalibrationData() {
  for (int i = 0; i < 3; i++) {
    motorCalibrationSerialData[i] = 0;
  }
  motorCalibrationPositionLetter = -1;
  motorCalibrationLateralLetter = -1;
  motorCalibrationSerialIndex = 0;
}

/* Mouvements **************************************************************************************/

/*
   Traitement des communication série pour les mouvements du drone
*/
void checkSerialMovements() {
  FrontLeftMotorMovementIndice = 0;
  FrontRightMotoMovementIndice = 0;
  BackLeftMotorMovementIndice = 0;
  BackRightMotoMovementIndice = 0;

  while (Serial.available() > 0)
  {
    char storageActionChar = Serial.read();

    if (storageActionChar == SERIAL_ACTION_STOP) {
      Stop = 1;
    }
    else if (storageActionChar == SERIAL_ACTION_ELEVATION || storageActionChar == SERIAL_ACTION_PICH || storageActionChar == SERIAL_ACTION_ROLL) {
      action = storageActionChar;
    } else if (storageActionChar == ';') {

      if (indexAction == 4) {
        manageSerialMovements();
      }

      clearSerialActionData();
    } else if (storageActionChar == SERIAL_ACTION_NEGATIF) {
      isNegatif = 1;
    }
    else {
      if (indexAction < 4)
      {
        value[indexAction] = storageActionChar;
        indexAction++;
        value[indexAction] = '\0';
      }
    }
  }

  manageSerialPich();
  manageSerialRoll();
}

/*
   Vide les valeurs de la communication serie de l'action
*/
void clearSerialActionData() {
  for (int i = 0; i < 3; i++) {
    value[i] = 0;
  }
  indexAction = 0;
  isNegatif = 0;
  action = -1;
}

/*
   Traitement des communication série pour la hauteur du drone
*/
void manageSerialMovements() {
  int valueNumeric = atoi(value);

  if (action == SERIAL_ACTION_ELEVATION) {
    ElevationIndice = valueNumeric;
  } else if (action == SERIAL_ACTION_PICH) {
    if (isNegatif == 0) {
      PichIndice = valueNumeric;
    } else {
      PichIndice = -valueNumeric;
    }
    ResetPichTimer = millis();

  } else if (action == SERIAL_ACTION_ROLL) {
    if (isNegatif == 0) {
      RollIndice = valueNumeric;
    } else {
      RollIndice = -valueNumeric;
    }

    ResetRollTimer = millis();
  }
}

/*
   Traitement des communication série pour le tangage du drone
*/
void manageSerialPich() {
  if (PichIndice != 0 && (millis() - ResetPichTimer) > 1000)
  {
    PichIndice = 0;
  }

  if (PichIndice > 0) {
    BackLeftMotorMovementIndice = PichIndice;
    BackRightMotoMovementIndice = PichIndice;
  } else if (PichIndice < 0) {
    FrontLeftMotorMovementIndice = -PichIndice;
    FrontRightMotoMovementIndice = -PichIndice;
  }
}

/*
   Traitement des communication série pour la roulis du drone
*/
void manageSerialRoll() {
  if (RollIndice != 0 && (millis() - ResetRollTimer) > 1000)
  {
    RollIndice = 0;
  }

  if (RollIndice > 0) {
    FrontLeftMotorMovementIndice = FrontLeftMotorMovementIndice + RollIndice;
    BackLeftMotorMovementIndice = BackLeftMotorMovementIndice + RollIndice;
  } else if (RollIndice < 0) {
    FrontRightMotoMovementIndice = FrontRightMotoMovementIndice - RollIndice;
    BackRightMotoMovementIndice = BackRightMotoMovementIndice - RollIndice;
  }
}

/* Stabilité **************************************************************************************/

/*
   Gère la stabilisation du drone dans la cas ou il n'est pas en mouvement tangage ou roulis
*/
void manageStability() {
  reinisialiseStability();
  pichStability();
  rollStability();
}

/*
   Remet à zéro les variables de stabilisation
*/
void reinisialiseStability() {
  FrontLeftMotorStabilisationIndice = 0;
  FrontRightMotorStabilisationIndice = 0;
  BackLeftMotorStabilisationIndice = 0;
  BackRightMotorStabilisationIndice = 0;
}

/*
   Gère la stabilisation du tangage du drone
*/
void pichStability() {
  if (PichIndice == 0) {

    int aPosition = getYAxe(1);

    if (aPosition < 0) {
      BackLeftMotorStabilisationIndice = -aPosition * 5;
      BackRightMotorStabilisationIndice = -aPosition * 5;
    } else if (aPosition > 0) {
      FrontLeftMotorStabilisationIndice = aPosition * 5;
      FrontRightMotorStabilisationIndice = aPosition * 5;
    }
  }
}

/*
   Gère la stabilisation du roulis du drone
*/
void rollStability() {
  if (RollIndice == 0) {
    int aPosition = getXAxe(1);

    if (aPosition < 0) {
      if (FrontLeftMotorStabilisationIndice < -aPosition) {
        FrontLeftMotorStabilisationIndice = -aPosition * 5;
      }
      if (BackLeftMotorStabilisationIndice < -aPosition) {
        BackLeftMotorStabilisationIndice = -aPosition * 5;
      }
    } else if (aPosition > 0) {
      if (FrontRightMotorStabilisationIndice < aPosition) {
        FrontRightMotorStabilisationIndice = aPosition * 5;
      }
      if (BackRightMotorStabilisationIndice < aPosition) {
        BackRightMotorStabilisationIndice = aPosition * 5;
      }
    }
  }
}

/* Set motors values **************************************************************************************/

/*
   Applique les valeurs aux moteurs
*/
int setMotorsValues() {

  if (ElevationIndice > 0) {
    //On met +5 pour éviter que les moteur tournent à moitier
    int FrontLeftMotorValue = FrontLeftMotorCalibrageIndice + ElevationIndice + FrontLeftMotorStabilisationIndice + FrontLeftMotorMovementIndice;
    int FrontRightMotorValue = FrontRightMotorCalibrageIndice + ElevationIndice + FrontRightMotorStabilisationIndice + FrontRightMotoMovementIndice;

    int BackLeftMotorValue = BackLeftMotorCalibrageIndice + ElevationIndice + BackLeftMotorStabilisationIndice + BackLeftMotorMovementIndice;
    int BackRightMotorValue = BackRightMotorCalibrageIndice + ElevationIndice + BackRightMotorStabilisationIndice + BackRightMotoMovementIndice;

    setFrontLeftMotorValues(FrontLeftMotorValue);
    setFrontRightMotorValues(FrontRightMotorValue);

    setBackLeftMotorValues(BackLeftMotorValue);
    setBackRightMotorValues(BackRightMotorValue);

    // DEBUG !!
    Serial.print(":");
    Serial.print(FrontLeftMotorValue);
    Serial.print("|");
    Serial.print(FrontRightMotorValue);
    Serial.print("|");
    Serial.print(BackLeftMotorValue);
    Serial.print("|");
    Serial.print(BackRightMotorValue);
    Serial.print(";");
  } else {
    //On met -10 pour éviter que les moteurs tournent à moitier
    setFrontLeftMotorValues(FrontLeftMotorCalibrageIndice - 10);
    setFrontRightMotorValues(FrontRightMotorCalibrageIndice - 10);

    setBackLeftMotorValues(BackLeftMotorCalibrageIndice - 10);
    setBackRightMotorValues(BackRightMotorCalibrageIndice - 10);

    // DEBUG !!
    /*Serial.print(":");
    Serial.print(FrontLeftMotorCalibrageIndice);
    Serial.print("|");
    Serial.print(FrontRightMotorCalibrageIndice);
    Serial.print("|");
    Serial.print(BackLeftMotorCalibrageIndice);
    Serial.print("|");
    Serial.print(BackRightMotorCalibrageIndice);
    Serial.print(";");*/
  }
}

/*
   Applique la valeur en paramètre au moteur avant gauche
*/
int setFrontLeftMotorValues(int value) {
  if (value != NextFrontLeftMotorValue) {

    escFrontLeft.writeMicroseconds(value);

    NextFrontLeftMotorValue = value;
  }
}

/*
   Applique la valeur en paramètre au moteur avant droite
*/
int setFrontRightMotorValues(int value) {
  if (value != NextFrontRightMotoValue) {

    escFrontRight.writeMicroseconds(value);

    NextFrontRightMotoValue = value;
  }
}

/*
   Applique la valeur en paramètre au moteur arrière gauche
*/
int setBackLeftMotorValues(int value) {
  if (value != NextBackLeftMotorValue) {

    escBackLeft.writeMicroseconds(value);

    NextBackLeftMotorValue = value;
  }
}

/*
   Applique la valeur en paramètre au moteur arrière droite
*/
int setBackRightMotorValues(int value) {
  if (value != NextBackRightMotoValue) {

    escBackRight.writeMicroseconds(value);

    NextBackRightMotoValue = value;
  }
}

/* Utile **************************************************************************************/

/*
   Récupère la valeur de l'axe X de l'accéléromètre. Si on met le paramètre à 1 on utilise le calibrage.
*/
int LastXAxeValue = 0; //Permet d'ignorer les changement de seulement +-1. Réduit effet vibration
int getXAxe(int withCalibration) {
  int x = 0;//analogRead(ACCELERMETER_X_PIN);
  if (withCalibration == 1) {
    x = x - XCalibrage;
    if (x + 1 == LastXAxeValue || x - 1 == LastXAxeValue) {
      x = LastXAxeValue;
    } else {
      LastXAxeValue = x;
    }
  }
  return x;
}

/*
   Récupère la valeur de l'axe Y de l'accéléromètre. Si on met le paramètre à 1 on utilise le calibrage.
*/
int LastYAxeValue = 0; //Permet d'ignorer les changement de seulement +-1. Réduit effet vibration
int getYAxe(int withCalibration) {
  int y = 0;//analogRead(ACCELERMETER_Y_PIN);
  if (withCalibration == 1) {
    y = y - YCalibrage;
    if (y + 1 == LastYAxeValue || y - 1 == LastYAxeValue) {
      y = LastYAxeValue;
    } else {
      LastYAxeValue = y;
    }
  }
  return y;
}

/*
   Envoie un message série de log
*/
void serialLog(char* This) {
  Serial.print("Log{");
  Serial.print(This);
  Serial.print("}");
}

/*
   Permet la comparaison d'une chaine de caractère avec ce qui est reçu de la communication série
*/
char Comp(char* This) {
  while (Serial.available() > 0) // Don't read unless
    // there you know there is data
  {
    if (index < 19) // One less than the size of the array
    {
      inChar = Serial.read(); // Read a character
      inData[index] = inChar; // Store it
      index++; // Increment where to write next
      inData[index] = '\0'; // Null terminate the string
    }
  }

  if (strcmp(inData, This) == 0) {
    for (int i = 0; i < 19; i++) {
      inData[i] = 0;
    }

    index = 0;
    return (1);
  }
  else {
    return (0);
  }
}
