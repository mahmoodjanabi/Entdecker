#define LEFT 0
#define RIGHT 1

// below can be changed, but should be PORTD pins;
// otherwise additional changes in the code are required
#define LEFT_ENC_PIN_A PD2  //pin 2
#define LEFT_ENC_PIN_B PD3  //pin 3

// below can be changed, but should be PORTC pins
// #define RIGHT_ENC_PIN_A PC4  //pin A4
// #define RIGHT_ENC_PIN_B PC5  //pin A5

volatile double left_speed = 0.0;
volatile double right_speed = 0.0;

static const signed char ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect) {
  static unsigned long last_time = 0;
  static long last_dt = 0;
  static unsigned char last_enc = 0;

  last_enc <<= 2; //shift previous state two places
  last_enc |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

  signed char stat = ENC_STATES[(last_enc & 0x0f)];

  if (stat != 0) {
    unsigned long t = micros();
    long dt = t - last_time;

    last_time = t;

    if (1000 < dt && dt < 150000) {
      dt *= stat;
      
      if (last_dt < 0 && dt > 0 || last_dt > 0 && dt < 0) {
        last_dt = 0;
      }
      
      last_dt = ((last_dt * 4) + dt) / 5;
      
      left_speed = 25000.0 / (double)last_dt;
    }
  }
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
// ISR (PCINT1_vect){
//   static unsigned long last_time = 0;
//   static long last_dt = 0;
//   static unsigned char last_enc = 0;
// 
//   last_enc <<= 2; //shift previous state two places
// 
//   last_enc |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
// 
//   signed char stat = ENC_STATES[(last_enc & 0x0f)];
// 
//   if (stat != 0) {
//     unsigned long t = micros();
//     long dt = t - last_time;
// 
//     last_time = t;
// 
//     if (1000 < dt && dt < 150000) {
//       dt *= stat;
//  
//       if (last_dt < 0 && dt > 0 || last_dt > 0 && dt < 0) {
//         last_dt = 0;
//       }
//       
//       last_dt = ((last_dt * 4) + dt) / 5;
//       
//       right_speed = 25000.0 / (double)last_dt;
//     }
//   }
// }

void initEncoder() {
  //set as inputs
  DDRD &= ~(1 << LEFT_ENC_PIN_A);
  DDRD &= ~(1 << LEFT_ENC_PIN_B);
//   DDRC &= ~(1 << RIGHT_ENC_PIN_A);
//   DDRC &= ~(1 << RIGHT_ENC_PIN_B);

  //enable pull up resistors
  PORTD |= (1 << LEFT_ENC_PIN_A);
  PORTD |= (1 << LEFT_ENC_PIN_B);
//   PORTC |= (1 << RIGHT_ENC_PIN_A);
//   PORTC |= (1 << RIGHT_ENC_PIN_B);

  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
//   PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
//  PCICR |= (1 << PCIE1) | (1 << PCIE2);
  PCICR |= 1 << PCIE2;
}

/* Wrap the encoder reading function */
double readSpeed(int i) {
  if (i == LEFT) {
    return left_speed;
  }
  else {
    return right_speed;
  }
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    left_speed = 0.0;
    return;
  }
  else {
    right_speed = 0.0;
    return;
  }
}

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

void setup() {
  Serial.begin(115200);

  initEncoder();
}

void loop() {
  Serial.print("left speed: ");
  Serial.println(left_speed);

  delay(500);
}
