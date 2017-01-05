#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DEBOUNCE_TIME 10
#define BUTTON_PIN_L 3
#define BUTTON_PIN_R 5
#define LED_PIN 13

// Adafruit_SSD1306 displayB(-1);
Adafruit_SSD1306 display1(-1);
Adafruit_SSD1306 display2(-1);
#define displayB display1

unsigned long changeTimeStampL = 0;
unsigned long changeTimeStampR = 0;
unsigned long lastOutputTimeStamp = 0;
unsigned long lastInput = 0;

char display_buffer[2][4][12];
unsigned char rgbBuffer[3];
unsigned char input_p;
char input_buffer[32];
unsigned char rgbPins[3] = { 9, 10, 11 };
unsigned char buttonState = 0;

boolean debounceStateL = 0;
boolean debounceStateR = 0;
boolean lastDisplay = false;

void setup() {
  unsigned char i;
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN_L, INPUT_PULLUP);
  pinMode(BUTTON_PIN_R, INPUT_PULLUP);

  for (i = 0; i < 3; i++) {
    pinMode(rgbPins[i], OUTPUT);
    analogWrite(rgbPins[i], 255);
  }

  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);

  display1.display();
  display2.display();
  delay(2000);

  displayB.clearDisplay();
  display1.display();
  display2.display();

  for (i = 0; i < 2; i++) {
    for (unsigned char k = 0; k < 4; k++) {
      for (unsigned char m = 0; m < 12; m++) {
        display_buffer[i][k][m] = 0;
      }
    }
  }

  for (i = 0; i < 32; i++) {
    input_buffer[i] = 0;
  }
  input_p = 0;

  Serial.print(F("Start - button: ")); Serial.println(digitalRead(BUTTON_PIN_L) << 1 | digitalRead(BUTTON_PIN_R));
}

void loop() {
  unsigned long timeStamp;
  unsigned char chr;
  boolean tempButtonL;
  boolean tempButtonR;
  boolean changed;
  
  while (Serial.available() > 0) {
    chr = Serial.read();

    lastInput = millis();

    if (chr == 10 || chr == 13) {
        input_buffer[input_p++] = 0;

      if (input_buffer[0] == 'o') {
        buffer_line(0);
      }
      else if (input_buffer[0] == 't') {
        buffer_line(1);
      }
      else if (input_buffer[0] == 'l') {
        buffer_led();
      }

      for (int i = 0; i < 32; i++) {
        input_buffer[i] = 0;
      }

      input_p = 0;
    }
    else {
      if (input_p < 32) {
        input_buffer[input_p++] = chr;
      }
    }
  }

  if (lastDisplay && (lastInput < millis() - 5000)) {
    clear_display();
  }

  tempButtonL = digitalRead(BUTTON_PIN_L);
  tempButtonR = digitalRead(BUTTON_PIN_R);
  changed = false;
  timeStamp = millis();

  if (tempButtonL != debounceStateL) {
    debounceStateL = tempButtonL;
    changeTimeStampL = timeStamp;
  }

  if (tempButtonR != debounceStateR) {
    debounceStateR = tempButtonR;
    changeTimeStampR = timeStamp;
  }

  if (changeTimeStampL < timeStamp - DEBOUNCE_TIME && debounceStateL != (buttonState >> 1)) {
    buttonState = (debounceStateL << 1) | (buttonState & 0x01);
    changed = true;
  }

  if (changeTimeStampR < timeStamp - DEBOUNCE_TIME && debounceStateR != (buttonState & 0x01)) {
    buttonState = debounceStateR | (buttonState & 0x02);
    changed = true;
  }

  if (changed) {
    Serial.println(buttonState);
    lastOutputTimeStamp = timeStamp;
  }
  else if (lastOutputTimeStamp < timeStamp - 1000) {
    Serial.println(buttonState);
    lastOutputTimeStamp = timeStamp;
  }

  if (buttonState > 0) {
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(LED_PIN, LOW);
  }
}

void buffer_line(unsigned char d) {
  unsigned char i;
  unsigned char l;
  
  if (input_p < 3 || d > 1) {
    return;
  }

  l = input_buffer[1] - 0x30;

  if (l > 3) {
    return;
  }

  for (i = 0; i < 12; i++) {
    display_buffer[d][l][i] = 0;
  }

  for (i = 0; i < 10 && input_buffer[i + 3] != 0; i++) {
    display_buffer[d][l][i] = input_buffer[i + 3];
  }

  displayB.clearDisplay();
  displayB.setCursor(0, 0);
  displayB.setTextSize(2);

  for (l = 0; l < 4; l++) {
    if (l % 2 == 0) {
      displayB.setTextColor(WHITE);
    }
    else {
      displayB.setTextColor(BLACK, WHITE);
    }
    displayB.println(display_buffer[d][l]);
  }

  if (d == 0) {
    display1.display();
  }
  else {
    display2.display();
  }

  for (i = 0; i < 3; i++) {
    analogWrite(rgbPins[i], 255 - rgbBuffer[i]);
  }

  lastDisplay = true;
}

void buffer_led() {
  unsigned char i;
  char *p = &(input_buffer[2]);
  char *str;

  for (i = 0; i < 3 && (str = strtok_r(p, " ", &p)) != '\0'; i++) {
    rgbBuffer[i] = atoi(str);
    analogWrite(rgbPins[i], 255 - rgbBuffer[i]);
  }

  lastDisplay = true;
}

void clear_display() {
  unsigned char d;
  unsigned char l;
  unsigned char i;
  
  for (d = 0; d < 2; d++) {
    for (l = 0; l < 4; l++) {
      for (i = 0; i < 12; i++) {
        display_buffer[d][l][i] = 0;
      }
    }
  }

  displayB.clearDisplay();
  display1.display();
  display2.display();

  for (i = 0; i < 3; i++) {
    analogWrite(rgbPins[i], 0);
  }

  analogWrite(rgbPins[0], 0);
  analogWrite(rgbPins[1], 255);
  analogWrite(rgbPins[2], 255);

  lastDisplay = false;
}

