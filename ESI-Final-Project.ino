/*------------------------------------------------------------*-
  Demonstration code for Group presentation 1
  (c) Minh-An Dao - Him Ko - Yue Qiao 2021
  
  reference:
  <https://github.com/arduino-libraries/Servo>
  <https://github.com/qub1750ul/Arduino_SoftwareReset>
  <https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib>
  
  version 1.00 - 02/09/2021 
---------------------------------------------------------------
 * For demonstration only
 *
 --------------------------------------------------------------*/
#include <avr/wdt.h>
#include "Servo.h"
#include "SPI.h"
#include "MFRC522.h"
#include "pitches.h"
#include <arduino-timer.h>
#include <SimpleKalmanFilter.h>

#define LDR_PIN           A0
#define BUTTON_PIN        3
#define PIR_PIN           4
#define SERVO_ONSWITCH    5
#define SERVO_OFFSWITCH   6
#define BUZZER_PIN        7
#define SOUND_SENSOR_PIN  8
#define RFID_RST_PIN      9  
#define RFID_SS_PIN       10
//      RFID_MOSI         11
//      RFID_MISO         12
//      RFID_SCK          13

// --- RFID configuration
#define NOTFOUND        -1
#define CARDS_KNOWN     2
#define CARD_SIZE       4
// card data
const byte RFID_KEY[CARDS_KNOWN][CARD_SIZE] = {
  {0xFA, 0x20, 0x7B, 0x2E},
  {0xe5, 0xa2, 0x9c, 0x2F}
};

// --- LDR components
#define LDR_THRESHOLD       600
/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty
 q: Process Noise
 */
SimpleKalmanFilter filter(2, 2, 0.01);

// --- servo angles
#define ON_TRIGGER          90
#define ON_RELEASE          0
#define ON_DEFAULT          ON_RELEASE
#define OFF_TRIGGER         90
#define OFF_RELEASE         0
#define OFF_DEFAULT         OFF_TRIGGER

enum {
  PIR,
  SOUND,
  NO_SENSOR
} system_state;

#define OFF                 0
#define ON                  1
#define ACTIVATE            2

#define PRESSED             0
#define RELEASED            1

uint8_t sound_val = 0;
uint16_t ldr_val = 0;
uint8_t  pir_val = 0;
uint8_t  id_buffer[CARD_SIZE];  // Matrix for storing new UID

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);  // Create MFRC522 instance
Servo on_servo, off_servo;                // Create 2 servo instances
Timer<2, millis> timer; // create a timer with 1 task and millisecond resolution

volatile uint8_t BUZZER_FLAG;
volatile uint8_t LIGHT_STATE;
volatile uint8_t button_state;
volatile uint8_t last_button_state;
bool routine(void *) {
  
  // --- Buzzer handler
  if (BUZZER_FLAG)
  {
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN)); // toggle the pin
    BUZZER_FLAG--;
  }
  // --- Button handler
  if (system_state==NO_SENSOR) //only trigger on NO_SENSOR state
  {
    button_state = digitalRead(BUTTON_PIN); //read button val
    if (last_button_state&&(last_button_state!=button_state))
    {
      if (LIGHT_STATE==ON)    switch_turnoff();
      else                    switch_turnon();
    }
    last_button_state = button_state; //reset last button state to the lates
  }
  return true;
}
// ---

void setup () 
{
  // --- System state initialize
  system_state = PIR;

  // --- Sensors initialize
  pinMode(SOUND_SENSOR_PIN,INPUT);
  pinMode(LDR_PIN,INPUT);
  pinMode(PIR_PIN,INPUT);
  digitalWrite(PIR_PIN,LOW);

  // --- rfid initialize
  SPI.begin();			  // Init SPI bus
	rfid.PCD_Init();		// Init MFRC522
  Serial.println("RFID Reader: "); rfid.PCD_DumpVersionToSerial();
  
  // --- servo initialize
  on_servo.attach(SERVO_ONSWITCH);
  off_servo.attach(SERVO_OFFSWITCH);
  on_servo.write(ON_DEFAULT); off_servo.write(OFF_DEFAULT);
  LIGHT_STATE = OFF;

  // --- button and buzzer initialize
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(BUZZER_PIN,OUTPUT);
  BUZZER_FLAG = OFF;
  button_state = digitalRead(BUTTON_PIN);
  last_button_state = button_state;
  timer.every(100, routine); //100ms
  
  // --- Serial initialize
  Serial.begin (9600);
  Serial.println("System ready!");
}
 
void loop() 
{
  timer.tick(); // tick the timer
  // ----------------- RFID data collection -----------
  if (getRFID()) {
    Serial.print("RFID:"); dump_byte_array(id_buffer,CARD_SIZE); Serial.println();
    checkUID(); // mode changing integrated inside
  }

  switch (system_state)
{
  case   PIR:
    // ----------------- LDR data collection -----------
    ldr_val = filter.updateEstimate(analogRead(LDR_PIN));
    // ----------------- PIR data collection -----------
    pir_val = digitalRead(PIR_PIN);
    Serial.print("LDR:"); Serial.print(ldr_val, DEC);
    Serial.print("\tPIR:"); Serial.println(pir_val, DEC);
    // ----------------- action on the switch -----------
    if ((ldr_val<LDR_THRESHOLD)&&(pir_val)) switch_turnon();
    else                                    switch_turnoff();
    break;
  case   SOUND:
    /* code */
    break;
  case   NO_SENSOR:
    /* do nothing - the timer already handle the task */
    break;
  default:
    break;
  }
  // // ----------------- Sound data collection -----------
  // sound_val = digitalRead(SOUND_SENSOR_PIN);
  // // Serial.print("Sound:");
  // Serial.println(sound_val, DEC);

  // if (LIGHT_STATE==ON)  switch_turnon();
  // else                  switch_turnoff();
}

// ----------------- Servo control -----------
void switch_turnon()
{
  on_servo.write(ON_TRIGGER);
  off_servo.write(OFF_RELEASE);
  LIGHT_STATE=ON;
}

void switch_turnoff()
{
  on_servo.write(ON_RELEASE);
  off_servo.write(OFF_TRIGGER);
  LIGHT_STATE=OFF;
}

// #################### Additional RFID helper functions ############################
bool getRFID(void) {
  bool isPICCpresent = false;
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial())
  { //if new card is found
    memcpy(id_buffer, rfid.uid.uidByte, CARD_SIZE); //save it to the buffer
    isPICCpresent = true; BUZZER_FLAG = ACTIVATE;
  }
  rfid.PICC_HaltA(); // Halt PICC
  rfid.PCD_StopCrypto1(); // Stop encryption on PCD
  return isPICCpresent;          // returns TRUE if PICC is detected, false otherwise
}//end getRFID

/*Routine for checking if the UID is correct*/
bool checkUID(void){
  for (byte i=0; i<CARDS_KNOWN; i++)
  {
    //counter for correct UID parts, there are CARD_SIZE parts in an UID, if 1 part correct, this counter will increase.
    //If all of them are correct (the UID is granted) then this counter will equal CARD_SIZE
    byte Rcounter=0; 
    for (byte j = 0; j < CARD_SIZE; j++){
      if (RFID_KEY[i][j]==id_buffer[j]) {Rcounter++;}
    }//end for
    if (Rcounter==4) { //access granted
      Serial.print(F("Mode updated to: "));
      system_state_update();
      return true;
    }//end if
  }
  return false;
}//end checkUID

void system_state_update() {
  switch (system_state)
  {
  case PIR:
    system_state = SOUND; Serial.println("SOUND mode");
    break;
  case SOUND:
    system_state = NO_SENSOR; Serial.println("NO SENSOR mode");
    break;
  case NO_SENSOR:
    system_state = PIR; Serial.println("PIR mode");
  default:
    break;
  }
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}