#include "MS5611.h"

namespace {
constexpr float mbar2mmHg = 0.7500616;
constexpr float mbar2Pa = 100.0;

constexpr float gravAcceleration = 9.81;

/*! \brief
 *  \param pressure Pressure in mmHg
 *  \param temperature Temperature in C
 *  \return Returns the density value in SI
 */
inline float getDensity(float pressure, float temperature) {
  return 0.125 * (1 + 0.0013 * (pressure - 760) - 0.0036 * (temperature - 15)) * gravAcceleration;
}
}  // end of nameless namespace

MS5611 STATIC(0x76);
MS5611 TOTAL(0x77);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin();
  bool one_begin = STATIC.begin();
  bool two_begin = TOTAL.begin();
  if (one_begin == true && two_begin == true) {
    Serial.println(F("Two MS5611 are found."));
  } else {
    if (!one_begin) {
      Serial.println(F("0x76 not found. Halt."));
    }
    if (!two_begin) {
      Serial.println(F("0x77 not found. Halt."));
    }

    while (1)
      ;
  }
  STATIC.setOversampling(OSR_STANDARD);
  TOTAL.setOversampling(OSR_STANDARD);
  Serial.println();
}


void loop() {
  int res = STATIC.read();
  if (res != MS5611_READ_OK) {
    Serial.print(F("0x76 Read err: "));
    Serial.println(res);
  }

  res = TOTAL.read();
  if (res != MS5611_READ_OK) {
    Serial.print(F("0x77 Read err: "));
    Serial.println(res);
  }

  float staticPressure = STATIC.getPressure();                              // in mbar
  float staticTemperature = STATIC.getTemperature();                        // in C

  float diffPressure = (TOTAL.getPressure() - staticPressure) * mbar2Pa;    // in Pa
  float avgTemperature = (staticTemperature + TOTAL.getTemperature()) / 2;  // in C
  staticPressure *= mbar2mmHg;                                              // in mmHg

  float airspeed = M_SQRT2 * sqrtf(diffPressure / getDensity(staticPressure, staticTemperature));

  Serial.print(F("$"));
  Serial.print(airspeed, 2);
  Serial.print(F(","));
  Serial.print(avgTemperature, 2);
  Serial.print(F(";"));
  delay(10);
}
