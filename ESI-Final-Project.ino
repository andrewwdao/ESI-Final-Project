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

#define LDR_PIN           A0
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
byte knownCards[CARDS_KNOWN][CARD_SIZE] = {
  {0x93, 0x47, 0xe0, 0x00},
  {0xe5, 0xa2, 0x9c, 0x2c}
};

char name[CARDS_KNOWN][10] = {
  "James",
  "Richard"
};

char password[CARDS_KNOWN][5] = {
  "1234", 
  "8888"
};


// --- servo angles
#define ON_DEFAULT          0
#define ON_TRIGGER          90
#define ON_RELEASE          0
#define OFF_DEFAULT         0
#define OFF_TRIGGER         180
#define OFF_RELEASE         0


uint8_t sound_val = 0;
uint16_t ldr_val = 0;
uint8_t  pir_val = 0;
uint8_t  id_buffer[CARD_SIZE];  // Matrix for storing new UID

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);  // Create MFRC522 instance
Servo on_servo, off_servo;                // 2 servo instances

Timer<1, millis> timer; // create a timer with 1 task and microsecond resolution
volatile uint8_t BUZZER_FLAG = 0;
bool toggle_buz(void *) {
  if (BUZZER_FLAG)
  {
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN)); // toggle the pin
    BUZZER_FLAG--;
  }
  return true;
}

void setup () 
{
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
  on_servo.write(0); off_servo.write(0);
  
  // --- buzzer initialize
  pinMode(BUZZER_PIN,OUTPUT);
  timer.every(100, toggle_buz); //100ms
  //
  Serial.begin (9600);
  Serial.println("System ready!");
}
 
void loop () 
{
  timer.tick(); // tick the timer
  
  // ----------------- LDR data collection -----------
  ldr_val = analogRead(LDR_PIN);
  Serial.print("LDR:"); Serial.println(ldr_val, DEC);
  // ----------------- Sound data collection -----------
  sound_val = digitalRead(SOUND_SENSOR_PIN);
  // Serial.print("Sound:");
  Serial.println(sound_val, DEC);
  // ----------------- PIR data collection -----------
  pir_val = digitalRead(PIR_PIN);
  Serial.print("PIR:"); Serial.println(pir_val, DEC);
  // ----------------- RFID data collection -----------
  if (getRFID()) {
    Serial.print("RFID:"); dump_byte_array(id_buffer,CARD_SIZE);
    Serial.println();
  }
  // ----------------- Servo control -----------
  on_servo.write(ON_TRIGGER); off_servo.write(OFF_TRIGGER);

  Serial.println();

}

// #################### Additional RFID helper functions ############################
bool getRFID() {
  bool isPICCpresent = false;
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial())
  { //if new card is found
    memcpy(id_buffer, rfid.uid.uidByte, CARD_SIZE); //save it to the buffer
    isPICCpresent = true; BUZZER_FLAG = 2;
  }
  rfid.PICC_HaltA(); // Halt PICC
  rfid.PCD_StopCrypto1(); // Stop encryption on PCD
  return isPICCpresent;          // returns TRUE if PICC is detected, false otherwise
}//end getRFID

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}
