#include <sx126x/Sx1262.hpp>

#include <math.h>

#include "sx126x/Assert.hpp"
#include <sx126x/IHal.hpp>

#include "InternalTypes.hpp"

namespace sx126x {

Sx1262::Sx1262(IHal& hal) noexcept
  : Sx126x{hal}
{
  // Note: this should really be "2", however, it seems that all SX1262 devices report as SX1261
  m_chipType = "SX1261";
}

Result
Sx1262::begin(const float frequency,
              const Bandwidth bandwidth,
              const SpreadingFactor spreadingFactor,
              const CodingRate codingRate,
              const uint8_t syncWord,
              const OutputPower outputPower,
              const uint16_t preambleLength,
              const TcxoVoltage tcxoVoltage,
              const bool useRegulatorLdo)
{
  // execute common part
  ASSERT(Sx126x::begin(codingRate, syncWord, preambleLength, tcxoVoltage, useRegulatorLdo));

  // configure publicly accessible settings
  ASSERT(setSpreadingFactor(spreadingFactor));
  ASSERT(setBandwidth(bandwidth));
  ASSERT(setFrequency(frequency));
  ASSERT(fixPaClamping());
  ASSERT(setOutputPower(outputPower));

  return Result::Ok;
}

Result
Sx1262::setFrequency(const float frequency, const bool skipCalibration)
{
  ASSERT_RANGE(frequency, 150.0F, 960.0F, Result::InvalidFrequency);

  // check if we need to recalibrate image
  // NOLINTNEXTLINE(*-type-promotion-in-math-fn)
  if (not skipCalibration and (fabsf(frequency - frequencyInMhz()) >= g_calibrateImageFrequencyTriggerMhz)) {
    ASSERT(calibrateImage(frequency));
  }

  // set frequency
  return setFrequencyRaw(frequency);
}

Result
Sx1262::setOutputPower(const OutputPower outputPower)
{
  // get current OCP configuration
  float currentLimit{0.0F};
  ASSERT(getCurrentLimit(&currentLimit));

  // set PA config
  ASSERT(setPaConfig(0x04, g_sx1262PaConfig));

  // set output power with default 200us ramp
  ASSERT(setTxParams(outputPower, PaRampTime::_200U));

  // restore OCP configuration
  ASSERT(setCurrentLimit(currentLimit));

  return Result::Ok;
}

} // namespace sx126x
