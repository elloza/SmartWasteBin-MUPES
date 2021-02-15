/*
  SmartBin

  INNOVACIÓN EN LA ESPECIALIDAD DE TECNOLOGÍA
  MUPES 2021

  ÁLVARO LOZANO MURCIEGO

*/

// BIBLIOTECAS NECESARIAS
#include <M5StickCPlus.h> // Biblioteca de la placa M5StickCPlus, es necesario instalarla desde un zip Info: https://docs.m5stack.com/#/en/core/m5stickc_plus
#include <ESP32Servo.h> // Biblioteca Servo.h pero para ESP32
#include <NewPing.h> // Ultrasonidos
#include <Wire.h> // Biblioteca Wire.h I2C para ToF https://docs.m5stack.com/#/en/unit/tof?id=schematic

#include <WiFi.h> // Biblioteca para el wifi
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros


// INFORMACIÓN DEL PUNTO DE ACCESO PARA CONECTARSE
char ssid[] = "Moderdonia2";   // your network SSID (name)
char pass[] = "12345678";   // your network password

WiFiClient  client;

unsigned long myChannelNumber = 1; // Este es el numero del canal
const char * myWriteAPIKey = "9YWL31OV4V652MCC"; // API KEY para escribir los datos

int number = 0;


// DEFINICIÓN DE PINES
#define PIN_SERVOMOTOR 26
#define TRIGGER_PIN  0// Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     36  // Arduino pin tied to echo pin on the ultrasonic sensor.

// DIRECCIONES DE MEMORIA PARA OBTENER INFORMACIÓN DEL TOF (
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define address 0x29

byte gbuf[16];


// ThingSpeak

#define TIME_BETWEEN_UPDATES 15000 // El mínimo de ThingSpeak es 15 segundos.
#define TIME_BETWEEN_UPDATES_UI 1000 // Tasa de refresco de la pantalla

// Variables utiles para el programa
const int MAX_DISTANCE = 100; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
float containerDistanceEmpty = 30.0; // Distancia del contenedor vacío.
int angle = 180;   // variable to hold the angle for the servo motor
int pos = 0;    // variable to store the servo position
int distance = 0;
int TOFDistante = 0;
float fillPercentage = 0.0;
unsigned long lastRequest = millis();
unsigned long lastMessageUpdate = millis();
char messageBuffer[40];


// Objetos para controlar ultrasonidos y servo
NewPing sonar = NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myServo;


void setup() {

  // PUERTO SERIE PARA DEBUG
  Serial.begin(115200); // open a serial connection to your computer

  //Connect to wifi
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);  // Initialize ThingSpeak


  // Inicializando la placa con la biblioteca M5
  M5.begin();
  M5.update();

  // Pantalla LCD
  M5.Lcd.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen( BLACK );

  // PRESENTACIÓN
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.fillScreen( BLACK );
  M5.Lcd.setCursor(50, 25);
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("Contenedor Inteligente");
  M5.Lcd.println("");
  M5.Lcd.setCursor(5, 50);
  M5.Lcd.println("Alvaro Lozano");
  M5.Lcd.setCursor(5, 75);
  M5.Lcd.println("Innovación Especialidad Tecnología");
  M5.Lcd.setCursor(5, 100);
  M5.Lcd.println("MUPES 2021");

  delay(5000);

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.fillScreen( BLACK );
 
  // ULTRASOINDOS
  pinMode(36, INPUT);
  gpio_pulldown_dis(GPIO_NUM_25);
  gpio_pullup_dis(GPIO_NUM_25);
  pinMode(0, OUTPUT);

  // SERVOMOTOR
  pinMode(PIN_SERVOMOTOR, OUTPUT);
  myServo.attach(PIN_SERVOMOTOR); // attaches the servo on pin 18 to the servo object

  // PEQUEÑA MODIFICACION PARA EL RUIDO DEL SPEAKER
  ledcDetachPin(SPEAKER_PIN);
  pinMode(SPEAKER_PIN, INPUT);

  // TOF
  //For HY2.0-4P  (GROVE CABLE) TOF
  Wire.begin();

}

void loop() {

  M5.update(); // need to call update()

  // SI PULSAN EL BOTON A, SE ABRE LA TAPA
  if (M5.BtnA.isPressed())
  {
    openSeconds(5);
  }

  // MEDIMOS DISTANCIA ULTRASONIDOS Y CALCULAMOS PORCENTAJE DE LLENADO
  distance = sonar.ping_cm();

  float filledDistance = containerDistanceEmpty - distance;
  fillPercentage = filledDistance * 100 / containerDistanceEmpty;


  // MEDIMOS DISTANCIA TOF PARA VER SI ES NECESARIO ABRIR TAPA
  TOFDistante = getTOFDistance();
  if (TOFDistante <= 100 && TOFDistante > 20) {
    openSeconds(5);
  }

  // MANDAMOS INFORMACIÓN A THINGSPEAK SI HA PASADO EL TIEMPO NECESARIO
  if (millis()-lastRequest > TIME_BETWEEN_UPDATES){
    sendDataToThingSpeak(fillPercentage);
    lastRequest = millis();
  }

  // PINTAMOS INFORMACIÓN CADA CIERTO TIEMPO EN LA PANTALLA
  if (millis()-lastMessageUpdate > TIME_BETWEEN_UPDATES_UI){
    sprintf(messageBuffer, "%.2f %% lleno",fillPercentage);
    showMessageLCD(messageBuffer);
    lastMessageUpdate = millis();
  }
  
  delay(15);
  
}


void showMessageLCD(char * message){
  
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.fillScreen( BLACK );
  M5.Lcd.setCursor(5, 35);
  M5.Lcd.setTextSize(2);
  M5.Lcd.println(message);
  
}


/***
 * 
 * FUNCIÓN QUE MANDA DATOS A THINGSPEAK
 * 
 */
void sendDataToThingSpeak(float percentage){

  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }

  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  Here, we write to field 1.
  int x = ThingSpeak.writeField(myChannelNumber, 1, percentage, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  showMessageLCD("ENVIANDO DATOS!");

}


/***
 * 
 * FUNCIÓN DISTANCIA DEL TOF
 * 
 */
int getTOFDistance() {

  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  byte val = 0;
  int cnt = 0;
  while (cnt < 100) { // 1 second waiting time max
    delay(10);
    val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    if (val & 0x01) break;
    cnt++;
  }
  if (val & 0x01) Serial.println("ready"); else Serial.println("not ready");

  read_block_data_at(0x14, 12);
  uint16_t acnt = makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist = makeuint16(gbuf[11], gbuf[10]);
  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
  
  Serial.print("distance ");       Serial.println(dist);
  return dist;

}


/**
 * 
 * FUNCIÓN PARA ABRIR LA TAPA N SECONDS
 * 
 */
void openSeconds(int seconds) {


  showMessageLCD("ABRIENDO...");

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  showMessageLCD("INTRODUZCA DESHECHO");
  delay(seconds * 1000);

  
  showMessageLCD("CERRANDO...");
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

}


/**
 * 
 * FUNCIONES PARA EL I2C
 * 
 * 
 */

uint16_t bswap(byte b[]) {
  // Big Endian unsigned short to little endian unsigned short
  uint16_t val = ((b[0] << 8) & b[1]);
  return val;
}

uint16_t makeuint16(int lsb, int msb) {
  return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void write_byte_data(byte data) {
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}

void write_byte_data_at(byte reg, byte data) {
  // write data word at address and register
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void write_word_data_at(byte reg, uint16_t data) {
  // write data word at address and register
  byte b0 = (data & 0xFF);
  byte b1 = ((data >> 8) && 0xFF);

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(b0);
  Wire.write(b1);
  Wire.endTransmission();
}

byte read_byte_data() {
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

byte read_byte_data_at(byte reg) {
  //write_byte_data((byte)0x00);
  write_byte_data(reg);
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  byte b = Wire.read();
  return b;
}

uint16_t read_word_data_at(byte reg) {
  write_byte_data(reg);
  Wire.requestFrom(address, 2);
  while (Wire.available() < 2) delay(1);
  gbuf[0] = Wire.read();
  gbuf[1] = Wire.read();
  return bswap(gbuf);
}

void read_block_data_at(byte reg, int sz) {
  int i = 0;
  write_byte_data(reg);
  Wire.requestFrom(address, sz);
  for (i = 0; i < sz; i++) {
    while (Wire.available() < 1) delay(1);
    gbuf[i] = Wire.read();
  }
}


uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  // Converts the encoded VCSEL period register value into the real
  // period in PLL clocks
  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}
