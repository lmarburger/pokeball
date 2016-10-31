#include <avr/sleep.h>
#include "RGBConverter.h"

#define BODS 7   //BOD Sleep bit in MCUCR
#define BODSE 2  //BOD Sleep enable bit in MCUCR

const int redLed = 4;
const int greenLed = 0;
const int blueLed = 1;
const int buttonLed = 3;

const int PWM_PIN_ON = 255;
const int PWM_PIN_OFF = 0;

// Registers for R G & B
volatile uint8_t* ports[] = {&OCR1B, &OCR0A, &OCR0B};
float R;
const int pwmIntervals = 100;
RGBConverter rgbConverter;

volatile bool interrupt = false;
volatile unsigned long lastInterrupt = 0;
volatile unsigned long startupMillis = 0;

unsigned long hibernateMillis = 0;
const int debounceDuration = 1000;
//const int hibernateTimeout = 5000;
const int hibernateTimeout = 1000;
const int runTimeout = 5000;


void setup() {
  pinSetup();
  pwmSetup();
  pulseSetup();
  enableInt0();
}

void loop() {
  handleInterrupt();
  handleHibernate();
  handleTimeout();

  if (hibernateMillis == 0) {
    pulseButton();
    hueCycle();
  } else {
    pwmSetColor(PWM_PIN_OFF, PWM_PIN_OFF, PWM_PIN_OFF);
    OCR1A = 1;
  }
}

void handleInterrupt() {
  if (interrupt) {
    interrupt = false;

    if (hibernateMillis == 0) {
      triggerHibernate();
    } else {
      resetHibernate();
    }
  }

  if (lastInterrupt > 0 && millis() >= lastInterrupt + debounceDuration) {
    lastInterrupt = 0;
    enableInt0();
  }
}

void handleHibernate() {
  if (hibernateMillis > 0 && millis() >= hibernateMillis) {
    hibernate();
    resetHibernate();
    interrupt = false;
  }
}

void handleTimeout() {
  if (startupMillis == 0) {
    startupMillis = millis();
  } else if (millis() >= startupMillis + runTimeout) {
    triggerHibernate();
  }
}

void triggerHibernate() {
  if (hibernateMillis == 0) {
    hibernateMillis = millis() + hibernateTimeout;
  }
}

void resetHibernate() {
  hibernateMillis = 0;
  startupMillis = 0;
}

void pinSetup() {
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
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(blueLed, LOW);
  digitalWrite(buttonLed, LOW);
}

void pwmSetup() {
  // Configure counter/timer0 for fast PWM on PB0 and PB1
  TCCR0A = 3<<COM0A0 | 3<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02 | 3<<CS00; // Optional; already set
  
  // Configure counter/timer1 for fast PWM on PB4
  TCCR1 = 1<<CTC1 | 1<<PWM1A | 3<<COM1A0 | 7<<CS10;
  GTCCR = 1<<PWM1B | 3<<COM1B0;

  // Interrupts on OC1A match and overflow
  TIMSK = TIMSK | 1<<OCIE1A | 1<<TOIE1;
}

void pulseSetup() {
  R = (pwmIntervals * log10(2)) / log10(255);
}

void pulseButton() {
  // Exponential brightness
  int brightness = (millis() % 3000) / (double)3000 * pwmIntervals + 1;
  if (brightness <= pwmIntervals / 2) {
    brightness = brightness * 2;
  } else {
    brightness = pwmIntervals - ((brightness - (pwmIntervals / 2)) * 2);
  }

  OCR1A = pow(2, (brightness / R));
}

void hueCycle() {
  double hue = (millis() / 20) % 1000 / (double)1000;
  byte rgb[3];
  rgbConverter.hsvToRgb(hue, 1, 1, rgb);
  pwmSetColor(rgb[0], rgb[1], rgb[2]);
}

// Sets color to specified intensity 0 (off) to 255 (max)
void pwmSetColor(int red, int green, int blue) {
  *ports[0] = PWM_PIN_ON - red;
  *ports[1] = PWM_PIN_ON - green;
  *ports[2] = PWM_PIN_ON - blue;
}

void hibernate() {
  byte adcsra, mcucr1, mcucr2;
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  adcsra = ADCSRA;                          //save ADCSRA
  ADCSRA &= ~_BV(ADEN);                     //disable ADC

  cli();                                    //stop interrupts to ensure the BOD timed sequence executes as required
                                            //and before modifying GIMSK which is also modified in the INT0 handler
  enableInt0();
  
  mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
  mcucr2 = mcucr1 & ~_BV(BODSE);            //if the MCU does not have BOD disable capability,
  MCUCR = mcucr1;                           //  this code has no effect
  MCUCR = mcucr2;
  
  sei();                                    //ensure interrupts enabled so we can wake up again

  sleep_cpu();                              //go to sleep
  sleep_disable();                          //wake up here
  
  ADCSRA = adcsra;                          //restore ADCSRA
}

void enableInt0() {
  MCUCR &= ~(_BV(ISC01) | _BV(ISC00));      //INT0 on low level
  GIMSK |= _BV(INT0);                       //enable INT0
  OCR1A = 255;
}

void disableInt0() {
  GIMSK &= ~_BV(INT0);                      //disable INT0
}

// External interrupt 0 wakes the MCU
ISR(INT0_vect) {
  disableInt0();
  interrupt = true;
  lastInterrupt = millis();
}

// Timer 1 compare compare match interrupt. Fires when timer equals OCR1A
ISR(TIMER1_COMPA_vect) {
  if (!bitRead(TIFR, TOV1)) {
    bitClear(PORTB, buttonLed);
  }
}

// Timer 1 overflow interrupt. Fires when timer overflows
ISR(TIMER1_OVF_vect) {
  bitSet(PORTB, buttonLed);
}
