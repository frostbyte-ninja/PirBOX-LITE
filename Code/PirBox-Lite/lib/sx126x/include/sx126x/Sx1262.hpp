#pragma once

#include <sx126x/Sx126x.hpp>

namespace sx126x {
class Sx1262 : public Sx126x
{
public:
  explicit Sx1262(IHal& hal) noexcept;

  Result begin(float frequency = 868.0F,
               Bandwidth bandwidth = Bandwidth::_125_0,
               SpreadingFactor spreadingFactor = SpreadingFactor::_9,
               CodingRate codingRate = CodingRate::_4_7,
               uint8_t syncWord = g_syncWordPrivate,
               OutputPower outputPower = OutputPower::_10Dbm,
               uint16_t preambleLength = 8,
               TcxoVoltage tcxoVoltage = TcxoVoltage::_1_6V,
               bool useRegulatorLdo = false);

  Result setFrequency(float frequency, bool skipCalibration = false);

  Result setOutputPower(OutputPower outputPower);
};
} // namespace sx126x
