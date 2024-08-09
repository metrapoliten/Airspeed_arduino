#include "MS5611.h"

namespace {
constexpr uint8_t kStaticAddr = 0x76; // поменять адреса местами, если неправильно
constexpr uint8_t kTotalAddr = 0x77;

/* \sigma (ς) */
constexpr float kCorrectionFactor{ 1. };  // поправочный коэффициент, необходимо поменять после опыта

constexpr float kMbar2mmHg{ 0.7500616 };
constexpr float kMbar2Pa{ 100. };

constexpr float kGravAcceleration{ 9.81 };
}

MS5611 STATIC{ kStaticAddr };
MS5611 TOTAL{ kTotalAddr };

namespace {

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


inline bool checkBegin(MS5611& dev, uint8_t addr)
{
  bool init{ dev.begin() };
  if (!init)
  {
    Serial.print(addr, HEX);
    Serial.println(F(" not found. Halt. Reason: "));
    if (!dev.isConnected())
    {
      Serial.println(F("not available on the I2C bus."));
    }
    else if (!dev.reset())
    {
      Serial.println(F("ROM could not be read."));
    }
    else {
      Serial.println(F("unhandled error."));
    }
    return false;
  }
  return true;
}

}  // end of nameless namespace

/* \varepsilon (ε) */
float compressibilityCorrection;  // поправка на сжимаемость потока

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin();
  
  if (!checkBegin(STATIC, kStaticAddr) or !checkBegin(TOTAL, kTotalAddr))
  {
    while(1);
  }
  Serial.println(F("Two MS5611 are found"));

  STATIC.setOversampling(OSR_STANDARD);
  TOTAL.setOversampling(OSR_STANDARD);
  Serial.println();
}


void loop() {
  checkRead();

  float staticPressure{ STATIC.getPressure() };        // in mbar
  float staticTemperature{ STATIC.getTemperature() };  // in °C

  float density{ getDensity(staticPressure * kMbar2mmHg, staticTemperature) };  // in kg/m^3

  float diffPressure{ getdiffPressure(staticPressure) };  // in Pa

  float airspeed{ getAirspeed(diffPressure, density, compressibilityCorrection) };

  Serial.println(airspeed, 2);

  compressibilityCorrection = getCompressibilityCorrection(airspeed);
}
