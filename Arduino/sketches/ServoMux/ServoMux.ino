#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define RPM_SENSOR 2

#define SERVO_CONTROL 4
#define SERVO_THROTTLE 6
#define SERVO_STEERING 5

#define CONTROL_MASTER 0
#define CONTROL_SLAVE 120
#define THROTTLE_NEUTRAL 90
#define THROTTLE_BRAKE   70    // 80
#define STEERING_VALUE 1446

#define LED_RED 3
#define LED_YELLOW 1
#define LED_GREEN 0

#define EXTRA_PIN 9

Servo servoControl;
Servo servoThrottle;
Servo servoSteering;

RF24 radio(7,8);
byte addresses[][6] = {"Entd1", "Entd2"};

#define COMMAND_LEN 8

char command_start[COMMAND_LEN] = { 'G', 'O', 0, 0, 0, 0, 0, 0 };
char command_stop[COMMAND_LEN] =  { 'S', 'T', 'O', 'P', 0, 0, 0, 0 };
char command_ok[COMMAND_LEN] =  { 'O', 'K', 0, 0, 0, 0, 0, 0 };

char rx_buff = 0;
unsigned long rx_time = 0;
unsigned long yellow_time = 0;
unsigned long go_time = 0;
unsigned long stop_time = 0;
bool going = false;
bool stopped = true;
unsigned long last_rpm_duration = -1;

void setup() {
  servoControl.attach(SERVO_CONTROL);
  servoThrottle.attach(SERVO_THROTTLE);
  servoSteering.attach(SERVO_STEERING);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);

  pinMode(RPM_SENSOR, INPUT_PULLUP);

  servoControl.write(CONTROL_MASTER);
  servoThrottle.write(THROTTLE_NEUTRAL);
  servoSteering.writeMicroseconds(STEERING_VALUE);
  delay(3000);
  servoThrottle.write(THROTTLE_BRAKE);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);

  pinMode(EXTRA_PIN, INPUT_PULLUP);

  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
//  radio.setRetries(0, 15);
  radio.setPayloadSize(COMMAND_LEN);
  radio.setAutoAck(true);

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();
}

void loop() {
  bool notBypass = digitalRead(EXTRA_PIN);
  unsigned long time = millis();

  if ((rx_time < time - 3000 || rx_buff != 'G') && notBypass) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);

    if (servoControl.read() != CONTROL_MASTER) {
      servoControl.write(CONTROL_MASTER);
    }

    if (going) {
      going = false;
      stopped = false;
      last_rpm_duration = -1;
      stop_time = time;
    }

    unsigned long dtime = stop_time - time;

    if (dtime < 200) {
      if (servoThrottle.read() != THROTTLE_NEUTRAL) {
        servoThrottle.write(THROTTLE_NEUTRAL);
      }
    }
    else {
      if (!stopped) {
        unsigned long duration = pulseIn(RPM_SENSOR, HIGH, 160000);

        if (duration > 0 && duration < 100000 &&
            (last_rpm_duration == -1 || last_rpm_duration <= duration + 50)) {
          last_rpm_duration = duration;
          digitalWrite(LED_GREEN, HIGH);

          if (servoThrottle.read() != THROTTLE_BRAKE) {
            servoThrottle.write(THROTTLE_BRAKE);
          }
        }
        else {
          stopped = true;
        }
      }

      if (stopped && servoThrottle.read() != THROTTLE_NEUTRAL) {
        servoThrottle.write(THROTTLE_NEUTRAL);
      }
    }
  }
  else {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);

    if (!going) {
      servoThrottle.write(THROTTLE_NEUTRAL);
      go_time = time;
      going = true;
    }

    unsigned long dtime = go_time - time;

    if (dtime > 200) {
      if (servoControl.read() != CONTROL_SLAVE) {
        servoControl.write(CONTROL_SLAVE);
      }
    }
  }

  if (rx_time > time - 500 ) {
    digitalWrite(LED_YELLOW, LOW);
    yellow_time = 0;
  }
  else {
    if (yellow_time == 0) {
      yellow_time = time;
    }

    if ((time - yellow_time) % 200 < 100) {
      digitalWrite(LED_YELLOW, HIGH);
    }
    else {
      digitalWrite(LED_YELLOW, LOW);
    }
  }

  if (radio.available()) {
    char received[COMMAND_LEN];

    while (radio.available()) {
      radio.read(received, COMMAND_LEN);
    }

    rx_time = millis();
    rx_buff = received[0];

    radio.stopListening();
    radio.write(command_ok, COMMAND_LEN);
    radio.startListening();
  }
}
