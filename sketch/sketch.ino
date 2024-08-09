#include "MS5611.h"

MS5611 STATIC{ 0x76 };  // поменять адреса местами, если неправильно
MS5611 TOTAL{ 0x77 };

/* \varepsilon (ε) */
float compressibilityCorrection;  // поправка на сжимаемость потока

namespace {
/* \sigma (ς) */
constexpr float kCorrectionFactor{ 1. };  // поправочный коэффициент, необходимо поменять после опыта

constexpr float kMbar2mmHg{ 0.7500616 };
constexpr float kMbar2Pa{ 100. };

constexpr float kGravAcceleration{ 9.81 };

inline void checkRead() {
  int res{ STATIC.read() };
  if (res != MS5611_READ_OK) {
    Serial.print(F("0x76 Read err: "));
    Serial.println(res);
  }

  res = TOTAL.read();
  if (res != MS5611_READ_OK) {
    Serial.print(F("0x77 Read err: "));
    Serial.println(res);
  }
}

/*! \brief
 *  \param pressure Pressure in mmHg
 *  \param temperature Temperature in C
 *  \return Returns the density value in SI (kg/m^3)
 */
inline float getDensity(float pressure, float temperature) {
  return 0.125 * (1 + 0.0013 * (pressure - 760) - 0.0036 * (temperature - 15)) * kGravAcceleration;
}

/*! \brief
 *  \param staticPressure Pressure in mbar
 *  \return Returns the differential pressure in Pa
 */
inline float getdiffPressure(float staticPressure) {
  return (TOTAL.getPressure() - staticPressure) * kMbar2Pa;
}
/*! \brief
 *  \param staticPressure Pressure in mbar
 *  \return Returns the differential pressure in Pa
 */
inline float getAirspeed(float diffPressure, float density, float compressibilityCorrection) {
  /* V = sqrt( 2 * ς * (p_1 - p_2) / [ρ * (1 + ε)]) */
  return M_SQRT2 * sqrtf(kCorrectionFactor * diffPressure / (density * (1 + compressibilityCorrection)));
}

/*! \brief
 *  \param airspeed airspeed
 *  \return Returns the compressibilityCorrection
 */
inline float getCompressibilityCorrection(float airspeed) {
  /*  ε = (M^2)/4 = (V / 330)^2 / 4 = V^2 / 435600 */
  return airspeed * airspeed / 435600;
}

}  // end of nameless namespace

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin();
  bool oneBegin{ STATIC.begin() };
  bool twoBegin{ TOTAL.begin() };
  if (oneBegin == true and twoBegin == true) {
    Serial.println(F("Two MS5611 are found."));
  } else {
    if (!oneBegin) {
      Serial.println(F("0x76 not found. Halt."));
    }
    if (!twoBegin) {
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
  checkRead();

  float staticPressure{ STATIC.getPressure() };        // in mbar
  float staticTemperature{ STATIC.getTemperature() };  // in °C

  float density{ getDensity(staticPressure, staticTemperature * kMbar2mmHg) };  // in kg/m^3

  float diffPressure{ getdiffPressure(staticPressure) };  // in Pa

  float airspeed{ getAirspeed(diffPressure, density, compressibilityCorrection) };

  Serial.println(airspeed, 2);

  compressibilityCorrection = getCompressibilityCorrection(airspeed);
}
