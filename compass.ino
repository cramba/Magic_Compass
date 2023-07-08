#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <hsv.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <AccelStepper.h>
#ifdef __AVR__
 #include <avr/power.h> 
#endif

/*******LED-Ring*******/
#define PIN        2 // Which pin on the Arduino is connected to the NeoPixels?
#define NUMPIXELS 24 // How many NeoPixels are attached to the Arduino?
#define LED_BRIGHTNESS 255     // general brightness of the LEDs from 0-255
#define MAXHUE 256*6
/*******Schrittmotor*******/
#define STEPS 2038
#define motorPin1  5      // IN1 on the ULN2003 driver
#define motorPin2  6      // IN2 on the ULN2003 driver
#define motorPin3  7     // IN3 on the ULN2003 driver
#define motorPin4  8     // IN4 on the ULN2003 driver
#define MotorInterfaceType 8
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);
#define DEGREESTOSTEPS 11.3777778
/*******Kompass-Modul*******/
HMC5883L compass;
/*******Hall-Sensor*******/
#define HALL_SENSOR 14
/*int hallSensorPin = 0;*/
int hallSensorValue = 0;
int hallSensorStatus = 0;
/*******Globale Variablen*******/
double steps = 0;
static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;
float ziel_breite = 0.0;
float ziel_laenge = 0.0;
float oldZielBreite = 0.0;
float oldZielLaenge = 0.0;
double distance_to_dest = 0.0;
double totaleDistanz = 0.0;
double totaleDistanzTEST = 0.0;
float strecke_pro_LED = 0.0;
float gelaufeneStrecke = 0.0;
int isTotalDistCalc = 0;
int LED_anzahl = 0;
int alte_LED_anzahl = 0;
float courseTo;
float degreesToTurn = 0.0;
float headingDegrees = 0.0, previousHeadingDegrees = 0.0;
byte gpsBlockEntered = 0;
byte destinationFound = 0;
byte firstDestRead = 0;
byte position = 0;
byte initColor = 0;

/*******RFID*******/
// configurable pinning
const uint8_t RST_PIN = 9; // reset pin
const uint8_t SS_PIN = 10; // serial data pin
const uint8_t BYTES_PER_BLOCK = 16; // usually either 16 or 32
MFRC522 mfrc522(SS_PIN, RST_PIN); // create MFRC522 instance
MFRC522::MIFARE_Key key; // access key

//boolean für totale Distanz Berechnug am Anfang
int calculatedTotalDistance;

//LED
Adafruit_NeoPixel led_Ring(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void ladebalken(){
  for (int i = 0; i < NUMPIXELS; i++)
      led_Ring.setPixelColor((i + position) % NUMPIXELS, getPixelColorHsv(i, 800, 255, led_Ring.gamma8(i * (255 / NUMPIXELS))));
  led_Ring.show();
  position++;
  position %= NUMPIXELS;
  delay(100);
  Serial.println("In Ladebalken");
}


void setup(){
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif

  Serial.begin(9600);
  ss.begin(GPSBaud);

  //LED
  led_Ring.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  led_Ring.show();
  led_Ring.setBrightness(30);

  //RFID
  SPI.begin(); // start SPI bus
  mfrc522.PCD_Init(); // init RFID module

  // initialize with default key value
  for (byte i = 0; i < MFRC522::MF_KEY_SIZE; i++) {
    key.keyByte[i] = 0xFF;
  }

  
  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin()){
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  
  
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(-205, -167);
  

  //----Motor Setup----
  // Set the maximum steps per second:
  stepper.setMaxSpeed(1000);
  
  // Set the maximum acceleration in steps per second^2:
  stepper.setAcceleration(10000);


  /********GPS Einstellung bis isValid()==true*********/
  while(!gps.location.isValid()) {
    Serial.println("GPS KACKE");
    gps.encode(ss.read());
    //LED-Ladebalken - blau
    //ladebalken();
  }
  /*
  led_Ring.fill(led_Ring.Color(0, 150, 0), 0, NUMPIXELS);
  led_Ring.show();
  delay(100);
  led_Ring.clear();
  led_Ring.show();
  */
  Serial.println("GPS GUT BRUDI");

  /******HallSensor******/
  pinMode(HALL_SENSOR, INPUT);
  
  hallSensorValue = digitalRead(HALL_SENSOR);
  Serial.print("1. Hall Sensor: "); Serial.println(hallSensorValue);
  while(hallSensorValue == 1){
    stepper.move(100);
    stepper.runToPosition();
    hallSensorValue = digitalRead(HALL_SENSOR);
    Serial.print("stepper's current position: "); Serial.println(stepper.currentPosition()%STEPS);
    Serial.print("Hall Sensor: "); Serial.println(hallSensorValue);
  }
  

  stepper.setCurrentPosition(0);

  Vector norm = compass.readNormalize();

  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (3.0 + (3.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
  if (heading < 0){
    heading += 2 * PI;
  }
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180/M_PI;

  if(headingDegrees <= 180){
    stepper.move((-headingDegrees) * DEGREESTOSTEPS);
  }else{
    stepper.move((360-headingDegrees) * DEGREESTOSTEPS);
  }
  stepper.runToPosition();
  stepper.setCurrentPosition(0);
  stepper.setAcceleration(400);
  previousHeadingDegrees = headingDegrees;

}

void loop(){
  /*
  //Ohne GPS-Signal: Nadel richtet sich nach Norden aus
  if(gpsBlockEntered == 0){
    //Kompass
    Vector norm = compass.readNormalize();

    // Calculate heading
    float heading = atan2(norm.YAxis, norm.XAxis);

    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    // (+) Positive or (-) for negative
    // For Bytom / Poland declination angle is 4'26E (positive)
    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
    float declinationAngle = (3.0 + (3.0 / 60.0)) / (180 / M_PI);
    heading += declinationAngle;

    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0){
      heading += 2 * PI;
    }
    if (heading > 2 * PI){
      heading -= 2 * PI;
    }

    // Convert to degrees
    headingDegrees = heading * 180/M_PI;

    // Output
    Serial.print(" Heading = "); Serial.print(heading);
    Serial.print(" Degress = "); Serial.println(headingDegrees); //Grad von Kompass, wo wir hinschauen

    //Nadel immer zum Norden ausrichten, und Nullpunkt aka Norden festhalten
    float difference = previousHeadingDegrees-headingDegrees;
    stepper.move(difference * DEGREESTOSTEPS);
    stepper.runToPosition();
    stepper.setAcceleration(400);

    previousHeadingDegrees = headingDegrees;

  }
  */
  if(initColor == 0){
    led_Ring.clear();
      led_Ring.fill(led_Ring.Color(0, 150, 0), 0, NUMPIXELS);
      led_Ring.show();
      delay(1000);
      led_Ring.clear();
      led_Ring.show();
      
      initColor = 1;
      
    }

    
  
  //RFID - ok
  //Autentifizierung
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    
    byte trailerBlock = 7; // stores the key
    MFRC522::StatusCode status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
    
      //union um int array im (IEEE Format) in float umzuwandeln
      union {
        uint8_t i[4];
        float f;
      } u;
  
      //ersten Wert von Block 5 lesen
      uint16_t blockAddress = 5; // value in [0,63]
      uint8_t bufferSize = BYTES_PER_BLOCK + 2;
      uint8_t buffer[bufferSize];
      status = mfrc522.MIFARE_Read(blockAddress, buffer, &bufferSize);
      // lesen erfolgreich
      // IEEE Format Buffer-Array in Union int-Array schreiben. Das Union-Float hat dann den selben Wert als Float
      Serial.println(F("Wert1:"));
      u.i[3] = buffer[3];
      u.i[2] = buffer[2];
      u.i[1] = buffer[1];
      u.i[0] = buffer[0];
      if(ziel_breite != u.f){
        oldZielBreite = ziel_breite;
        destinationFound = 0;
      }
      ziel_breite = u.f;
      Serial.print("RFID-Breite: "); Serial.println(ziel_breite);

      
      //zweiten Wert von Block 6 lesen
      blockAddress = 6; // value in [0,63]
      uint8_t buffer2[bufferSize];
      status = mfrc522.MIFARE_Read(blockAddress, buffer2, &bufferSize);
      // lesen erfolgreich
      // IEEE Format Buffer-Array in Union int-Array schreiben. Das Union-Float hat dann den selben Wert als Float
      Serial.println(F("Wert2:"));
      u.i[3] = buffer2[3];
      u.i[2] = buffer2[2];
      u.i[1] = buffer2[1];
      u.i[0] = buffer2[0];
      if(ziel_laenge != u.f){
        oldZielLaenge = ziel_laenge;
        destinationFound = 0;
      }
      ziel_laenge = u.f;
      Serial.print("RFID-Laenge: "); Serial.println(ziel_laenge);

      //GPS-Daten-Prints
      printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
      printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
      printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
      printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
      printInt(gps.location.age(), gps.location.isValid(), 5);
      printDateTime(gps.date, gps.time);
      printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
      printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
      printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
      printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

      //totale Distanz angeben
      totaleDistanz = gps.distanceBetween(gps.location.lat(), gps.location.lng(), ziel_breite, ziel_laenge);
      Serial.println(gps.location.lat());
      Serial.println(gps.location.lng());      
      strecke_pro_LED = totaleDistanz / NUMPIXELS;

      //LED Signal - BLAUES AUFLEUCHTEN
      led_Ring.fill(led_Ring.Color(0, 0, 150), 0, NUMPIXELS);
      led_Ring.show();

      //Lade-Balken erstellen

      //Ziel noch nicht gefunden
      if(destinationFound == 0){
          courseTo = gps.courseTo(gps.location.lat(), gps.location.lng(), ziel_breite, ziel_laenge);
          stepper.move(courseTo * DEGREESTOSTEPS);
          stepper.runToPosition();
          stepper.setAcceleration(400);
          Serial.println("ERSTES ZIEL GEFUNDEN!");
          destinationFound = 1;
          firstDestRead = 1;
        }else{
        /*if(gps.location.isUpdated()){
          float oldCourseTo = gps.courseTo(gps.location.lat(), gps.location.lng(), oldZielBreite, oldZielLaenge);
          courseTo = gps.courseTo(gps.location.lat(), gps.location.lng(), ziel_breite, ziel_laenge);
          float difference = courseTo - oldCourseTo;
  
          stepper.move(difference * DEGREESTOSTEPS);
          stepper.runToPosition();
          stepper.setAcceleration(400);
          destinationFound = 1;
          Serial.println("ZIEL GEFUNDEN!");
        }*/
        Serial.println("DestinationFound = 0 -> REINGEGANGEN");
        gps.encode(ss.read());
          
          hallSensorValue = digitalRead(HALL_SENSOR);
          Serial.print("1. Hall Sensor: "); Serial.println(hallSensorValue);
          hallSensorValue = 1;
          while(hallSensorValue == 1){
            stepper.move(100);
            stepper.runToPosition();
            hallSensorValue = digitalRead(HALL_SENSOR);
            Serial.print("stepper's current position: "); Serial.println(stepper.currentPosition()%STEPS);
            Serial.print("Hall Sensor: "); Serial.println(hallSensorValue);
          }
          stepper.setCurrentPosition(0);
          Vector norm = compass.readNormalize();
          float heading = atan2(norm.YAxis, norm.XAxis);
          float declinationAngle = (3.0 + (3.0 / 60.0)) / (180 / M_PI);
          heading += declinationAngle;
          if (heading < 0){
            heading += 2 * PI;
          }
          if (heading > 2 * PI){
            heading -= 2 * PI;
          }
          // Convert to degrees
          float headingDegrees = heading * 180/M_PI;
        
          if(headingDegrees <= 180){
            stepper.move((-headingDegrees) * DEGREESTOSTEPS);
          }else{
            stepper.move((360-headingDegrees) * DEGREESTOSTEPS);
          }
          stepper.runToPosition();
          stepper.setCurrentPosition(0);
          stepper.setAcceleration(400);
          previousHeadingDegrees = headingDegrees;
        
      }
      
      mfrc522.PCD_Init();
  }

  /*
  //GPS-Signal: Nadel richtet sich zum Ziel aus
  while(ss.available() > 0 && destinationFound == 0 && ziel_laenge != 0.0 && ziel_breite != 0.0){
      stepper.move(courseTo * DEGREESTOSTEPS);
      stepper.runToPosition();
      stepper.setAcceleration(400);
      destinationFound = 1;
  }
  */
  
  // This sketch displays information every time a new sentence is correctly encoded.
  while ((ss.available() > 0) && (destinationFound > 0)){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      float latitude, longitude;
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      Serial.print("Latitude = "); Serial.print(latitude);
      Serial.print(" Longitude = "); Serial.println(longitude);
      Serial.print("Distanz zwischen Ziel und hier: ");
      distance_to_dest = gps.distanceBetween(latitude, longitude, ziel_breite, ziel_laenge);
      if(totaleDistanz == 0.0){
        totaleDistanz = distance_to_dest;
        strecke_pro_LED = totaleDistanz/24;
        Serial.print("Step Size: "); Serial.println(strecke_pro_LED);
      }
      Serial.println(distance_to_dest);
      Serial.print("Totale Distanz: "); Serial.println(totaleDistanz);

      //LEDs zum Leuchten bringen
      float gelaufeneStrecke = totaleDistanz - distance_to_dest;
      Serial.print("Gelaufene Strecke: "); Serial.println(gelaufeneStrecke);

      alte_LED_anzahl = LED_anzahl;
      LED_anzahl = gelaufeneStrecke / strecke_pro_LED;
      Serial.print("LED-Anzahl: "); Serial.println(LED_anzahl);
      //LED-Ring wird ausgeschaltet
      led_Ring.clear();
      led_Ring.show();
      //neue LED-Aufleuchtung wird gesetzt
      if(LED_anzahl > 0){
        led_Ring.fill(led_Ring.Color(0, 150, 0), 0, LED_anzahl);
        led_Ring.show();
      }else if(LED_anzahl < 0){
        led_Ring.fill(led_Ring.Color(150, 0, 0), (NUMPIXELS - 1)+LED_anzahl, LED_anzahl*(-1));
        led_Ring.show();
      }
      if(distance_to_dest > 2*totaleDistanz){
        led_Ring.fill(led_Ring.Color(150, 0, 0), 0, NUMPIXELS);
        led_Ring.show();
        delay(1000);
        led_Ring.clear();
        led_Ring.show();
      }
      if(distance_to_dest < 5.0){
        uint8_t r, g, b;
        for(float offset = 0.0; offset<PI*5; offset+=0.4) {
          r = (uint8_t)((sin(offset*1.0) + 1)/2 * LED_BRIGHTNESS);
          g = (uint8_t)((sin(offset*1.5) + 1)/2 * LED_BRIGHTNESS);
          b = (uint8_t)((sin(offset*2.0) + 1)/2 * LED_BRIGHTNESS);
          for(byte i=0; i<NUMPIXELS; i++) {
            led_Ring.setPixelColor(i, led_Ring.Color(r, g, b));
          }             
          led_Ring.show();
          delay(100);
        }
      }
      //Kursrichtung ermitteln
      courseTo = gps.courseTo(latitude, longitude, ziel_breite, ziel_laenge);
      Serial.print("courseTo: "); Serial.println(courseTo);

      //Nadel zum Ziel ausrichten
      //degreesToTurn = courseTo - headingDegrees;
      //stepper.move(courseTo * DEGREESTOSTEPS);

      //Kompass
      Vector norm = compass.readNormalize();

      // Calculate heading
      float heading = atan2(norm.YAxis, norm.XAxis);

      // Set declination angle on your location and fix heading
      // You can find your declination on: http://magnetic-declination.com/
      // (+) Positive or (-) for negative
      // For Bytom / Poland declination angle is 4'26E (positive)
      // Formula: (deg + (min / 60.0)) / (180 / M_PI);
      float declinationAngle = (3.0 + (3.0 / 60.0)) / (180 / M_PI);
      heading += declinationAngle;

      // Correct for heading < 0deg and heading > 360deg
      if (heading < 0){
        heading += 2 * PI;
      }
      if (heading > 2 * PI){
        heading -= 2 * PI;
      }

      // Convert to degrees
      headingDegrees = heading * 180/M_PI;
      Serial.print("HeadingDegrees: "); Serial.println(headingDegrees);
      float difference = previousHeadingDegrees-headingDegrees;
      
      /*
      if(difference > 180){
        difference = 360 - difference;
      }else if(difference < (-180)){
        difference += 360;
      }
      */

      Serial.print("Difference (Heading/OldHeading): "); Serial.println(difference);

      //if(difference >= 10 || difference <= (-10)){
        stepper.move(difference * DEGREESTOSTEPS);
        stepper.runToPosition();
        stepper.setAcceleration(400);
        previousHeadingDegrees = headingDegrees;
      //}

      

      gpsBlockEntered = 1;
      
    }
  }
  




    //alter Code
  /*

    //Zielrichtung - eigene Schau-Richtung = Position, an die sich Nadel drehen muss
    degreesToTurn = courseTo - headingDegrees;
    Serial.print("Nadelausrichtung: "); Serial.println(degreesToTurn);


    float diff = stepper.currentPosition() - degreesToTurn;
    
    //Schrittmotor dreht sich nur, wenn er sich mehr als 20 Grad drehen soll
    if(diff > 20){
      if(degreesToTurn < 180){
        steps = degreesToTurn * 11.3777778;
      }else{
        steps = degreesToTurn * (-11.3777778);
      }
      stepper.moveTo(steps);
      stepper.runToPosition();
    }else if(diff < -20){
      if(degreesToTurn > -180){
        steps = degreesToTurn * 11.3777778;
      }else{
        steps = degreesToTurn * (-11.3777778);
      }
    }

    
    if(headingDegrees <= (180)) {
      steps = headingDegrees * 11.3777778;
    }else{
      steps = ((360) - headingDegrees) * -11.3777778;
    }
    
    //----Turn motor----;
    //If this part is commented out I have no problems with the compass sensor
    
    steps = degreesToTurn * 11.3777778;
    stepper.moveTo(steps);
    stepper.runToPosition();
    
    delay(100);
    }
    Serial.print("stepper.currentPosition nach Schalter: "); Serial.println(stepper.currentPosition());
    
  }
  */  
  

}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

  static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
