#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace driver {
#include <sx126x.h>
} // namespace driver

#include <sx126x/Types.hpp>

namespace sx126x {
enum class DeviceErrors : uint16_t
{
  PaRamp = 0b100000000U,
  PllLock = 0b001000000U,
  XoscStart = 0b000100000U,
  ImgCalib = 0b000010000U,
  AdcCalib = 0b000001000U,
  PllCalib = 0b000000100U,
  Rc13mCalib = 0b000000010U,
  Rc64kCalib = 0b000000001U,
};

constexpr uint8_t g_calibrateImage430Mhz1{0x6BU};
constexpr uint8_t g_calibrateImage430Mhz2{0x6FU};
constexpr uint8_t g_calibrateImage470Mhz1{0x75U};
constexpr uint8_t g_calibrateImage470Mhz2{0x81U};
constexpr uint8_t g_calibrateImage779Mhz1{0xC1U};
constexpr uint8_t g_calibrateImage779Mhz2{0xC5U};
constexpr uint8_t g_calibrateImage863Mhz1{0xD7U};
constexpr uint8_t g_calibrateImage863Mhz2{0xDBU};
constexpr uint8_t g_calibrateImage902Mhz1{0xE1U};
constexpr uint8_t g_calibrateImage902Mhz2{0xE9U};

constexpr size_t g_maxPacketLength{255U};
constexpr uint8_t g_cadParamDetMin{10U};
constexpr uint32_t g_txNoTimeout{0U};

constexpr float g_calibrateImageFrequencyTriggerMhz{20.0F};
constexpr uint8_t g_sx1262PaConfig{0x00};
constexpr uint16_t g_versionRegisterAddress{0x0320};

constexpr auto
toResult(driver::sx126x_status_t value) noexcept
{
  return static_cast<Result>(value);
}

constexpr bool
hasError(const DeviceErrors mask, const DeviceErrors test) noexcept
{
  return (to_underlying(mask) & to_underlying(test)) != 0;
}

constexpr auto
operator+(const CodingRate value) noexcept
{
  return to_underlying(value);
}

constexpr auto
operator+(const OutputPower value) noexcept
{
  return to_underlying(value);
}

constexpr auto
operator+(const SpreadingFactor value) noexcept
{
  return to_underlying(value);
}

inline uint16_t
toKhz(const Bandwidth bandwidth) noexcept
{
  return static_cast<uint16_t>(driver::sx126x_get_lora_bw_in_hz(static_cast<driver::sx126x_lora_bw_t>(bandwidth)) /
                               1000U);
}

constexpr uint8_t
calibrationFrequencyMin(const float frequency) noexcept
{
  auto coefMin = static_cast<uint8_t>(floor((frequency - 1.0) / 4.0));

  if ((coefMin & 1U) == 0) {
    --coefMin;
  }
  return coefMin;
}

constexpr uint8_t
calibrationFrequencyMax(const float frequency) noexcept
{
  auto coefMax = static_cast<uint8_t>(ceil((frequency + 1.0) / 4.0));

  if ((coefMax & 1U) == 0) {
    ++coefMax;
  }
  return coefMax;
}

} // namespace sx126x
