#include <Servo.h> 
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define SERVO_CONTROL 4
#define SERVO_THROTTLE 5
#define SERVO_STEERING 6

#define LED_RED 3
#define LED_YELLOW 1
#define LED_GREEN 0

#define EXTRA_PIN 9

Servo servoControl;
Servo servoThrottle;
Servo servoSteering;

RF24 radio(7,8);
uint8_t address[] = { "radio" };

#define COMMAND_LEN 8

char command_start[COMMAND_LEN] = { 'G', 'O', 0, 0, 0, 0, 0, 0 };
char command_stop[COMMAND_LEN] =  { 'S', 'T', 'O', 'P', 0, 0, 0, 0 };
char command_ok[COMMAND_LEN] =  { 'O', 'K', 0, 0, 0, 0, 0, 0 };

volatile char rx_buff = 0;
volatile unsigned long rx_time = 0;
volatile uint32_t round_trip_timer = 0;
unsigned long yellow_time = 0;

uint8_t pong = 'O';

void setup() {
  servoControl.attach(SERVO_CONTROL);
  servoThrottle.attach(SERVO_THROTTLE);
  servoSteering.attach(SERVO_STEERING);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  
  servoThrottle.write(90);
  servoSteering.write(90);

  pinMode(EXTRA_PIN, INPUT_PULLUP);
  
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.enableDynamicPayloads();
  
  radio.openWritingPipe(address);
  radio.openReadingPipe(1,address); 
  radio.startListening();
  
  delay(50);
  attachInterrupt(0, check_radio, LOW);
} 

void loop() {
  int extraValue = digitalRead(EXTRA_PIN);
  unsigned long time = millis();
  
  if ((rx_time < time - 3000 || rx_buff != 'G') && extraValue) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
  
    servoControl.write(0);
  }
  else if (rx_time < time - 500 && extraValue) {
    if (yellow_time < time - 100) {
      digitalWrite(LED_YELLOW, HIGH);
      yellow_time = time;
    }
    else {
      digitalWrite(LED_YELLOW, LOW);
    }
  }
  else {
    servoControl.write(120);
    
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, LOW);
  }

  delay(25);
}

void check_radio(void) {
  bool tx, fail, rx;
  radio.whatHappened(tx, fail, rx);

  if (rx || radio.available()) {
    if(radio.getDynamicPayloadSize() < 1){
      // Corrupt payload has been flushed
      return;
    }

    char received[COMMAND_LEN];
    radio.read(received, COMMAND_LEN);

    rx_time = millis();
    rx_buff = received[0];

    radio.stopListening();
    delayMicroseconds(100);

    radio.startWrite(command_ok, COMMAND_LEN, 0);
  }

  if (tx || fail) {
     radio.startListening();
  }
}

