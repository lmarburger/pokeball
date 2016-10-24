#include <avr/sleep.h>

#define BODS 7   //BOD Sleep bit in MCUCR
#define BODSE 2  //BOD Sleep enable bit in MCUCR

const int redLed = 0;
const int greenLed = 1;
const int blueLed = 4;
const int buttonLed = 3;

bool randSeeded = false;

void setup() {
  //to minimize power consumption while sleeping, output pins must not source
  //or sink any current. input pins must have a defined level; a good way to
  //ensure this is to enable the internal pullup resistors.

  //make all pins inputs with pullups enabled
  for (byte i=0; i<5; i++) {
    pinMode(i, INPUT);
    digitalWrite(i, HIGH);
  }
  
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(buttonLed, OUTPUT);

  flash(HIGH, HIGH, HIGH, 500);
 }

void loop() {
  hibernate();
  capture();
}

void capture() {
  if (!randSeeded) {
    randomSeed(millis());
    randSeeded = true;
  }

  for (int i = 0; i < 3; i++) {
    flash(HIGH, HIGH, HIGH, 500);
    delay(500);
  }
  
  int rand = random(0, 100);
  if (rand < 10) {         // Ran away
    flash(HIGH, LOW, LOW, 2000);
  } else if (rand < 40) {  // Unsuccessful
    flash(LOW, LOW, HIGH, 2000);
  } else {                 // Caught!
    flash(LOW, HIGH, LOW, 2000);
  }
}

void setColor(int red, int green, int blue) {
  digitalWrite(redLed, red);
  digitalWrite(greenLed, green);
  digitalWrite(blueLed, blue);  
}

void flash(int red, int green, int blue, int delayMs) {
  setColor(red, green, blue);
  digitalWrite(buttonLed, HIGH);
  
  delay(delayMs);
  
  setColor(LOW, LOW, LOW);
  digitalWrite(buttonLed, LOW);
}

void hibernate(void) {
  byte adcsra, mcucr1, mcucr2;
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  adcsra = ADCSRA;                          //save ADCSRA
  ADCSRA &= ~_BV(ADEN);                     //disable ADC

  cli();                                    //stop interrupts to ensure the BOD timed sequence executes as required
                                            //and before modifying GIMSK which is also modified in the INT0 handler
  
  MCUCR &= ~(_BV(ISC01) | _BV(ISC00));      //INT0 on low level
  GIMSK |= _BV(INT0);                       //enable INT0
  
  mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
  mcucr2 = mcucr1 & ~_BV(BODSE);            //if the MCU does not have BOD disable capability,
  MCUCR = mcucr1;                           //  this code has no effect
  MCUCR = mcucr2;
  
  sei();                                    //ensure interrupts enabled so we can wake up again

  sleep_cpu();                              //go to sleep
  sleep_disable();                          //wake up here
  
  ADCSRA = adcsra;                          //restore ADCSRA
}

//external interrupt 0 wakes the MCU
ISR(INT0_vect) {
  GIMSK = 0;  //disable external interrupts (only need one to wake up)
}
