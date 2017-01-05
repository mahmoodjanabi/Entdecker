#include <SPI.h>
#include "RF24.h"

RF24 radio(7,8);
byte addresses[][6] = {"Entd1", "Entd2"};

#define COMMAND_LEN 8

char command_start[COMMAND_LEN] = { 'G', 'O', 0, 0, 0, 0, 0, 0 };
char command_stop[COMMAND_LEN] =  { 'S', 'T', 'O', 'P', 0, 0, 0, 0 };
char command_ok[COMMAND_LEN] =  { 'O', 'K', 0, 0, 0, 0, 0, 0 };

unsigned long round_trip_timer = 0;
unsigned long yellow_time = 0;
unsigned long rx_time = 0;
bool yellow_on = false;

#define BUTTON_PIN 3
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

#define LED_RED 4
#define LED_YELLOW 5
#define LED_GREEN 6

void setup() {
//  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);

  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
//  radio.setRetries(0, 15);
  radio.setPayloadSize(COMMAND_LEN);
  radio.setAutoAck(true);

  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.startListening();
}

void loop() {
  unsigned long time_start = millis();
  int reading = digitalRead(BUTTON_PIN);
  char *p;

  if (reading != lastButtonState) {
    lastDebounceTime = time_start;
  }
  lastButtonState = reading;

  if ((time_start - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
    }
  }

  if (!buttonState) {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);

//    Serial.println("Sending go");
    p = command_start;
  }
  else {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);

//    Serial.println("Sending stop");
    p = command_stop;
  }

  radio.stopListening();
  radio.write(p, COMMAND_LEN);
  radio.startListening();

  delayMicroseconds(500);

  if (radio.available()) {
    char received[COMMAND_LEN];

    while (radio.available()) {
      radio.read(received, COMMAND_LEN);
    }

    rx_time = millis();

    if (received[0] == 'O') {
      digitalWrite(LED_YELLOW, LOW);
      yellow_on = false;

//      Serial.print("Received OK, Round Trip Time: ");
    }
    else {
      if (!yellow_on) {
        yellow_on = true;
        yellow_time = millis();
      }

//      Serial.print("Received ? , Round Trip Time: ");
    }

//    Serial.println(round_trip_timer);
  }

  unsigned long time = millis();

  if (rx_time < time - 500 && !yellow_on) {
    yellow_on = true;
    yellow_time = time;
  }

  if (yellow_on) {
    if ((millis() - yellow_time) % 200 < 100) {
      digitalWrite(LED_YELLOW, HIGH);
    }
    else {
      digitalWrite(LED_YELLOW, LOW);
    }
  }

  unsigned long time_stop = millis();

  if (time_stop - time_start < 50) {
    delay(50 + time_start - time_stop);
  }
}
