#pragma once

#include <stddef.h>
#include <stdint.h>

#include <etl/type_traits.h>

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <string>
using String = std::string;
#endif

namespace driver {
#include <sx126x.h>
} // namespace driver

namespace sx126x {

namespace detail {
template<typename TEnum, typename = void>
struct underlying_type_impl;

template<typename TEnum>
struct underlying_type_impl<TEnum, etl::enable_if_t<etl::is_enum<TEnum>::value>>
{
  using Type = __underlying_type(TEnum);
};
} // namespace detail

template<typename TEnum>
// NOLINTNEXTLINE(*-identifier-naming)
using underlying_type_t = typename detail::underlying_type_impl<etl::remove_cv_t<TEnum>>::Type;

template<typename TEnum, etl::enable_if_t<etl::is_enum_v<TEnum>>* = nullptr>
constexpr auto
to_underlying(TEnum value) noexcept // NOLINT(*-identifier-naming)
{
  return static_cast<underlying_type_t<TEnum>>(value);
}

enum class Result : int16_t
{
  Error = 3, // sx126x_status_t::SX126X_STATUS_ERROR
  UnknownValue = 2, // sx126x_status_t::SX126X_STATUS_UNKNOWN_VALUE
  UnsupportedFeature = 1, // sx126x_status_t::SX126X_STATUS_UNSUPPORTED_FEATURE
  Ok = 0, // sx126x_status_t::SX126X_STATUS_OK and RADIOLIB_ERR_NONE
  ChipNotFound = -2,
  MemoryAllocationFailed = -3,
  PacketTooLong = -4,
  TxTimeout = -5,
  RxTimeout = -6,
  CrcMismatch = -7,
  InvalidFrequency = -12,
  InvalidCurrentLimit = -17,
  WrongModem = -20,
  Unsupported = -25,
  SpiCmdTimeout = -705,
};

enum class Standby : uint8_t
{
  Rc = 0x00U,
  Xosc = 0x01U,
};

enum class RadioMode : uint8_t
{
  None = 0U,
  Rx,
  Tx,
};

enum class TcxoVoltage : uint8_t
{
  _1_6V = 0x00U,
  _1_7V = 0x01U,
  _1_8V = 0x02U,
  _2_2V = 0x03U,
  _2_4V = 0x04U,
  _2_7V = 0x05U,
  _3_0V = 0x06U,
  _3_3V = 0x07U,
  _0V = 0xFFU,
};

enum class Bandwidth : uint8_t
{
  _7_8 = 0x00U,
  _10_4 = 0x08U,
  _15_6 = 0x01U,
  _20_8 = 0x09U,
  _31_25 = 0x02U,
  _41_7 = 0x0AU,
  _62_5 = 0x03U,
  _125_0 = 0x04U,
  _250_0 = 0x05U,
  _500_0 = 0x06U,
};

enum class CodingRate : uint8_t
{
  _4_5 = 0x01U,
  _4_6 = 0x02U,
  _4_7 = 0x03U,
  _4_8 = 0x04U,
};

enum class SpreadingFactor : uint8_t
{
  _5 = 5U,
  _6 = 6U,
  _7 = 7U,
  _8 = 8U,
  _9 = 9U,
  _10 = 10U,
  _11 = 11U,
  _12 = 12U,
};

enum class OutputPower : int8_t
{
  _neg9Dbm = -9,
  _neg8Dbm = -8,
  _neg7Dbm = -7,
  _neg6Dbm = -6,
  _neg5Dbm = -5,
  _neg4Dbm = -4,
  _neg3Dbm = -3,
  _neg2Dbm = -2,
  _neg1Dbm = -1,
  _0Dbm = 0,
  _1Dbm = 1,
  _2Dbm = 2,
  _3Dbm = 3,
  _4Dbm = 4,
  _5Dbm = 5,
  _6Dbm = 6,
  _7Dbm = 7,
  _8Dbm = 8,
  _9Dbm = 9,
  _10Dbm = 10,
  _11Dbm = 11,
  _12Dbm = 12,
  _13Dbm = 13,
  _14Dbm = 14,
  _15Dbm = 15,
  _16Dbm = 16,
  _17Dbm = 17,
  _18Dbm = 18,
  _19Dbm = 19,
  _20Dbm = 20,
  _21Dbm = 21,
  _22Dbm = 22,
};

enum class PacketLengthMode : uint8_t
{
  Explicit = 0x00U,
  Implicit = 0x01U,
};

enum class PaRampTime : uint8_t
{
  _10U = 0x00,
  _20U = 0x01,
  _40U = 0x02,
  _80U = 0x03,
  _200U = 0x04,
  _800U = 0x05,
  _1700U = 0x06,
  _3400U = 0x07,
};

enum class IrqFlags : uint16_t
// clang-format off
{
  None             = 0b000000000000000U, // no interrupts
  TxDone           = 0b000000000000001U, // packet transmission completed
  RxDone           = 0b000000000000010U, // packet received
  PreambleDetected = 0b000000000000100U, // preamble detected
  SyncWordValid    = 0b000000000001000U, // valid sync word detected
  HeaderValid      = 0b000000000010000U, // valid LoRa header received
  HeaderError      = 0b000000000100000U, // LoRa header CRC error
  CrcError         = 0b000000001000000U, // wrong CRC received
  CadDone          = 0b000000010000000U, // channel activity detection finished
  CadDetected      = 0b000000100000000U, // channel activity detected
  Timeout          = 0b000001000000000U, // Rx or Tx timeout
  LrFhssHop        = 0b100000000000000U, // PA ramped up during LR-FHSS hop
  All              = 0b100001111111111U, // bits 0–9 & 14 = all interrupts
};
// clang-format on

constexpr auto
operator+(const IrqFlags value) noexcept
{
  return to_underlying(value);
}

constexpr auto
operator|(const IrqFlags lhs, const IrqFlags rhs) noexcept
{
  return static_cast<IrqFlags>(+lhs | +rhs);
}

inline bool
hasFlag(const IrqFlags mask, const IrqFlags test) noexcept
{
  return (+mask & +test) != 0U;
}

struct ReceiveConfig
{
  uint32_t timeout;
  IrqFlags irqFlags;
  IrqFlags irqMask;
  size_t len;
};

struct TransmitConfig
{
  const uint8_t* data;
  size_t len;
};

union RadioModeConfig
{
  ReceiveConfig receive;
  TransmitConfig transmit;
};

struct PacketStatus
{
  int8_t rssiPktInDbm; //!< RSSI of the last packet
  int8_t snrPktInDb; //!< SNR of the last packet
  int8_t signalRssiPktInDbm; //!< Estimation of RSSI (after despreading)
};

using Time = unsigned long; // NOLINT(*-runtime-int)

constexpr IrqFlags g_rxDefaultFlags{IrqFlags::RxDone | IrqFlags::Timeout | IrqFlags::CrcError | IrqFlags::HeaderValid |
                                    IrqFlags::HeaderError};
constexpr auto g_rxDefaultMask{IrqFlags::RxDone};
constexpr uint32_t g_rxTimeoutNone{SX126X_RX_SINGLE_MODE};
constexpr uint32_t g_rxTimeoutInf{SX126X_RX_CONTINUOUS};

constexpr uint8_t g_syncWordPublic{0x34};
constexpr uint8_t g_syncWordPrivate{0x12};

} // namespace sx126x
