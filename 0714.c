#include "HX711.h"

#define DT1 3
#define SCK1 2

#define DT2 5
#define SCK2 4 

HX711 scale1;
HX711 scale2;

// 這兩個要自己校正
float calibration_factor1 = 255.0;
float calibration_factor2 = 255.0;

void setup() {
  Serial.begin(115200);

  scale1.begin(DT1, SCK1);
  scale2.begin(DT2, SCK2);

  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);

  delay(1000);

  scale1.tare(20);
  scale2.tare(20);
}

void loop() {
  float force1 = scale1.get_units(5);
  float force2 = scale2.get_units(5);

  Serial.print(force1);
  Serial.print(",");
  Serial.println(-force2);

  delay(100);
}
