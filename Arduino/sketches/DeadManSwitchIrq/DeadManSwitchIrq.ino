#include <SPI.h>
#include "RF24.h"

RF24 radio(7,8);
uint8_t address[] = { "radio" };

#define COMMAND_LEN 8

char command_start[COMMAND_LEN] = { 'G', 'O', 0, 0, 0, 0, 0, 0 };
char command_stop[COMMAND_LEN] =  { 'S', 'T', 'O', 'P', 0, 0, 0, 0 };

volatile uint32_t round_trip_timer = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(5, INPUT_PULLUP);
  
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
  int value = digitalRead(5);
  
  if (!value) {
    Serial.print("Sending go");
    radio.stopListening();                
    round_trip_timer = micros();
    radio.startWrite(command_start, COMMAND_LEN,0);
  }
  else {
    Serial.print("Sending stop");
    radio.stopListening();                
    round_trip_timer = micros();
    radio.startWrite(command_stop, COMMAND_LEN,0);
  }
  
  delay(50);
}

void check_radio(void) {
  bool tx,fail,rx;
  
  radio.whatHappened(tx,fail,rx);

  if (rx) {
    if (radio.getDynamicPayloadSize() < 1) {
      return;
    }
    
    char received[COMMAND_LEN];
    radio.read(received, COMMAND_LEN);

    round_trip_timer = micros() - round_trip_timer;

    if (received[0] == 'O') {
      Serial.print(F("Received OK, Round Trip Time: "));
    }
    else {
      Serial.print(F("Received ? , Round Trip Time: "));
    }
    
    Serial.println(round_trip_timer);    
  }


  if (tx || fail) {
    radio.startListening(); 
    Serial.println(tx ? ":OK" : ":Fail");
  }
}
