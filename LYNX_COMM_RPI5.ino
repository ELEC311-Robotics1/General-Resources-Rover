#include <avr/wdt.h>
#include <Encoder.h>
#include <Sabertooth.h>

Encoder encFR(18, A8);
Encoder encRR(19, A9);
Encoder encFL(20, A10);
Encoder encRL(21, A11);

Sabertooth ST(128, Serial3);

int datum, sel;

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  delay(500);
  ST.motor(1, 0);
  ST.motor(2, 0);
  Serial.println(0);  // signal ready
}

void loop() {
  if (Serial.available() > 0) {
    sel = Serial.read();

    // Left forward
    if (sel == 1) {
      while (!Serial.available());
      datum = Serial.read();
      ST.motor(2, datum);
    }
    // Left backward
    else if (sel == 2) {
      while (!Serial.available());
      datum = Serial.read();
      ST.motor(2, -datum);
    }
    // Right forward
    else if (sel == 3) {
      while (!Serial.available());
      datum = Serial.read();
      ST.motor(1, datum);
    }
    // Right backward
    else if (sel == 4) {
      while (!Serial.available());
      datum = Serial.read();
      ST.motor(1, -datum);
    }
    // Left encoders
    else if (sel == 5) {
      Serial.print(encFL.read()); Serial.print(' ');
      Serial.println(encRL.read());
    }
    // Right encoders
    else if (sel == 6) {
      Serial.print(encFR.read()); Serial.print(' ');
      Serial.println(encRR.read());
    }
    // Watchdog reset
    else if (sel == 7) {
      ST.motor(1, 0);
      ST.motor(2, 0);
      delay(100);
      wdt_enable(WDTO_15MS);
      while (1);
    }
  }
}