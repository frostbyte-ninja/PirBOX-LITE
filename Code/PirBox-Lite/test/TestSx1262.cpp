#include <chrono>
#include <cstddef>

#include <gtest/gtest.h>

namespace driver {
#include <sx126x.h>
}

#include <sx126x_hal.h>

#include <sx126x/Sx1262.hpp>

#include "Mocks.hpp"

using namespace ::testing;

namespace {
constexpr std::uint8_t chipSelectPin{1U};
constexpr std::uint8_t resetPin{2U};
constexpr std::uint8_t busyPin{3U};
constexpr std::uint8_t irqPin{4U};
constexpr std::uint8_t gpioModeInput{111U};
constexpr std::uint8_t gpioModeOutput{112U};
constexpr std::uint8_t gpioLevelLow{113U};
constexpr std::uint8_t gpioLevelHigh{114U};
constexpr std::uint8_t interruptRising{115U};
constexpr std::uint8_t interruptFalling{116U};

enum class sx126x_commands_t
{
  // Operational Modes Functions
  SX126X_SET_SLEEP = 0x84,
  SX126X_SET_STANDBY = 0x80,
  SX126X_SET_FS = 0xC1,
  SX126X_SET_TX = 0x83,
  SX126X_SET_RX = 0x82,
  SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
  SX126X_SET_RX_DUTY_CYCLE = 0x94,
  SX126X_SET_CAD = 0xC5,
  SX126X_SET_TX_CONTINUOUS_WAVE = 0xD1,
  SX126X_SET_TX_INFINITE_PREAMBLE = 0xD2,
  SX126X_SET_REGULATOR_MODE = 0x96,
  SX126X_CALIBRATE = 0x89,
  SX126X_CALIBRATE_IMAGE = 0x98,
  SX126X_SET_PA_CFG = 0x95,
  SX126X_SET_RX_TX_FALLBACK_MODE = 0x93,
  // Registers and buffer Access
  SX126X_WRITE_REGISTER = 0x0D,
  SX126X_READ_REGISTER = 0x1D,
  SX126X_WRITE_BUFFER = 0x0E,
  SX126X_READ_BUFFER = 0x1E,
  // DIO and IRQ Control Functions
  SX126X_SET_DIO_IRQ_PARAMS = 0x08,
  SX126X_GET_IRQ_STATUS = 0x12,
  SX126X_CLR_IRQ_STATUS = 0x02,
  SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
  SX126X_SET_DIO3_AS_TCXO_CTRL = 0x97,
  // RF Modulation and Packet-Related Functions
  SX126X_SET_RF_FREQUENCY = 0x86,
  SX126X_SET_PKT_TYPE = 0x8A,
  SX126X_GET_PKT_TYPE = 0x11,
  SX126X_SET_TX_PARAMS = 0x8E,
  SX126X_SET_MODULATION_PARAMS = 0x8B,
  SX126X_SET_PKT_PARAMS = 0x8C,
  SX126X_SET_CAD_PARAMS = 0x88,
  SX126X_SET_BUFFER_BASE_ADDRESS = 0x8F,
  SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
  // Communication Status Information
  SX126X_GET_STATUS = 0xC0,
  SX126X_GET_RX_BUFFER_STATUS = 0x13,
  SX126X_GET_PKT_STATUS = 0x14,
  SX126X_GET_RSSI_INST = 0x15,
  SX126X_GET_STATS = 0x10,
  SX126X_RESET_STATS = 0x00,
  // Miscellaneous
  SX126X_GET_DEVICE_ERRORS = 0x17,
  SX126X_CLR_DEVICE_ERRORS = 0x07,
};

constexpr auto
operator+(sx126x_commands_t value) noexcept
{
  return sx126x::to_underlying(value);
}

} // namespace

class Sx1262 : public sx126x::Sx1262
{
public:
  using sx126x::Sx1262::Sx1262;

  using sx126x::Sx1262::clearIrqStatus;
};

class Sx1262_Test : public Test
{
public:
  Sx1262_Test()
  {
    EXPECT_CALL(m_hal, chipSelectPin).WillRepeatedly(Return(chipSelectPin));
    EXPECT_CALL(m_hal, resetPin).WillRepeatedly(Return(resetPin));
    EXPECT_CALL(m_hal, busyPin).WillRepeatedly(Return(busyPin));
    EXPECT_CALL(m_hal, irqPin).WillRepeatedly(Return(irqPin));
    EXPECT_CALL(m_hal, gpioModeInput).WillRepeatedly(Return(gpioModeInput));
    EXPECT_CALL(m_hal, gpioModeOutput).WillRepeatedly(Return(gpioModeOutput));
    EXPECT_CALL(m_hal, gpioLevelLow).WillRepeatedly(Return(gpioLevelLow));
    EXPECT_CALL(m_hal, gpioLevelHigh).WillRepeatedly(Return(gpioLevelHigh));
    EXPECT_CALL(m_hal, interruptRising).WillRepeatedly(Return(interruptRising));
    EXPECT_CALL(m_hal, interruptFalling).WillRepeatedly(Return(interruptFalling));

    EXPECT_CALL(m_hal, pinToInterrupt(_)).WillRepeatedly(ReturnArg<0>());

    EXPECT_CALL(m_hal, milliseconds).WillRepeatedly(Invoke([this] {
      const auto now = std::chrono::high_resolution_clock::now();
      return std::chrono::duration_cast<std::chrono::milliseconds>(now - m_start).count();
    }));
  }

protected:
  template<std::size_t N>
  void expectHalWrite(const uint8_t (&array)[N], bool waitForRadio = true)
  {
    EXPECT_CALL(m_hal, write(_, N, nullptr, 0, waitForRadio))
      .With(::testing::Args<0, 1>(ElementsAreArray(array, N)))
      .WillOnce(Return(sx126x_hal_status_t::SX126X_HAL_STATUS_OK));
  }

  NiceMock<sx126x::MockHal> m_hal;
  Sx1262 uut{m_hal};

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> m_start{std::chrono::high_resolution_clock::now()};
};

TEST_F(Sx1262_Test, begin)
{
  // todo
}

TEST_F(Sx1262_Test, reset_verify)
{
  {
    InSequence seq;
    EXPECT_CALL(m_hal, pinMode(resetPin, gpioModeOutput)).Times(1);
    EXPECT_CALL(m_hal, digitalWrite(resetPin, gpioLevelLow)).Times(1);
    EXPECT_CALL(m_hal, sleepMs(_)).Times(1);
    EXPECT_CALL(m_hal, digitalWrite(resetPin, gpioLevelHigh)).Times(1);
  }

  expectHalWrite({
    +sx126x_commands_t::SX126X_SET_STANDBY,
    driver::sx126x_standby_cfgs_t::SX126X_STANDBY_CFG_RC,
  });

  uut.reset(true);
}

TEST_F(Sx1262_Test, reset_no_verify)
{
  {
    InSequence seq;
    EXPECT_CALL(m_hal, pinMode(resetPin, gpioModeOutput)).Times(1);
    EXPECT_CALL(m_hal, digitalWrite(resetPin, gpioLevelLow)).Times(1);
    EXPECT_CALL(m_hal, sleepMs(_)).Times(1);
    EXPECT_CALL(m_hal, digitalWrite(resetPin, gpioLevelHigh)).Times(1);
  }

  EXPECT_CALL(m_hal, write(_, _, _, _, _)).Times(0);

  uut.reset(false);
}

TEST_F(Sx1262_Test, sleep_warmstart)
{
  expectHalWrite(
    {
      +sx126x_commands_t::SX126X_SET_SLEEP,
      driver::sx126x_sleep_cfgs_t::SX126X_SLEEP_CFG_WARM_START,
    },
    false);

  uut.sleep(true);
}

TEST_F(Sx1262_Test, sleep_coldstart)
{
  expectHalWrite(
    {
      +sx126x_commands_t::SX126X_SET_SLEEP,
      driver::sx126x_sleep_cfgs_t::SX126X_SLEEP_CFG_COLD_START,
    },
    false);

  uut.sleep(false);
}

TEST_F(Sx1262_Test, standby)
{
  expectHalWrite({
    +sx126x_commands_t::SX126X_SET_STANDBY,
    driver::sx126x_standby_cfgs_t::SX126X_STANDBY_CFG_RC,
  });

  uut.standby(sx126x::Standby::Rc, false);
}

TEST_F(Sx1262_Test, standby_wakeup)
{
  InSequence seq;

  EXPECT_CALL(m_hal, wakeup).WillOnce(Return(sx126x_hal_status_t::SX126X_HAL_STATUS_OK));

  expectHalWrite({
    +sx126x_commands_t::SX126X_SET_STANDBY,
    driver::sx126x_standby_cfgs_t::SX126X_STANDBY_CFG_XOSC,
  });

  uut.standby(sx126x::Standby::Xosc, true);
}

TEST_F(Sx1262_Test, setPacketReceivedOrSentAction)
{
  const auto callback = reinterpret_cast<sx126x::IHal::CallBack>(0xDEADBEEF);
  EXPECT_CALL(m_hal, attachInterrupt(irqPin, callback, interruptRising)).Times(1);

  uut.setPacketReceivedOrSentAction(callback);
}

TEST_F(Sx1262_Test, clearPacketReceivedOrSentAction)
{
  EXPECT_CALL(m_hal, detachInterrupt(irqPin)).Times(1);

  uut.clearPacketReceivedOrSentAction();
}

TEST_F(Sx1262_Test, transmit)
{
  // todo
}

TEST_F(Sx1262_Test, startTransmit)
{
  // todo
}

TEST_F(Sx1262_Test, transmitDirect)
{
  // todo
}

TEST_F(Sx1262_Test, finishTransmit)
{
  // todo
}

TEST_F(Sx1262_Test, receive)
{
  // todo
}

TEST_F(Sx1262_Test, startReceive)
{
  // todo
}

TEST_F(Sx1262_Test, readData)
{
  // todo
}

TEST_F(Sx1262_Test, stageMode)
{
  // todo
}

TEST_F(Sx1262_Test, launchMode)
{
  // todo
}

TEST_F(Sx1262_Test, setBandwidth)
{
  // todo
}

TEST_F(Sx1262_Test, setSpreadingFactor)
{
  // todo
}

TEST_F(Sx1262_Test, setCodingRate)
{
  // todo
}

TEST_F(Sx1262_Test, setSyncWord)
{
  // todo
}

TEST_F(Sx1262_Test, setCurrentLimit)
{
  // todo
}

TEST_F(Sx1262_Test, getCurrentLimit)
{
  // todo
}

TEST_F(Sx1262_Test, setPreambleLength)
{
  // todo
}

TEST_F(Sx1262_Test, setCrc)
{
  // todo
}

TEST_F(Sx1262_Test, setTcxO)
{
  // todo
}

TEST_F(Sx1262_Test, setDio2AsRfSwitch)
{
  // todo
}

TEST_F(Sx1262_Test, getDataRate)
{
  // todo
}

TEST_F(Sx1262_Test, getRssi)
{
  // todo
}

TEST_F(Sx1262_Test, getSnr)
{
  // todo
}

TEST_F(Sx1262_Test, getPacketLength)
{
  // todo
}

TEST_F(Sx1262_Test, getTimeOnAirInMs)
{
  EXPECT_EQ(uut.getTimeOnAirInMs(0), 20);
  EXPECT_EQ(uut.getTimeOnAirInMs(10), 35);
  EXPECT_EQ(uut.getTimeOnAirInMs(50), 99);
  EXPECT_EQ(uut.getTimeOnAirInMs(100), 178);
  EXPECT_EQ(uut.getTimeOnAirInMs(150), 257);
  EXPECT_EQ(uut.getTimeOnAirInMs(200), 336);
  EXPECT_EQ(uut.getTimeOnAirInMs(250), 414);
  EXPECT_EQ(uut.getTimeOnAirInMs(255), 422);
}

TEST_F(Sx1262_Test, getIrqFlags)
{
  // todo
}

TEST_F(Sx1262_Test, clearIrqFlags)
{
  // todo
}

TEST_F(Sx1262_Test, implicitHeader)
{
  // todo
}

TEST_F(Sx1262_Test, explicitHeader)
{
  // todo
}

TEST_F(Sx1262_Test, setRegulatorLdo)
{
  // todo
}

TEST_F(Sx1262_Test, setRegulatorDcdc)
{
  // todo
}

TEST_F(Sx1262_Test, forceLdro)
{
  // todo
}

TEST_F(Sx1262_Test, autoLdro)
{
  // todo
}

TEST_F(Sx1262_Test, invertIq)
{
  // todo
}

TEST_F(Sx1262_Test, setPaConfig)
{
  // todo
}

TEST_F(Sx1262_Test, calibrateImage)
{
  // todo
}

TEST_F(Sx1262_Test, setTx)
{
  // todo
}

TEST_F(Sx1262_Test, setRx)
{
  // todo
}

TEST_F(Sx1262_Test, writeBuffer)
{
  // todo
}

TEST_F(Sx1262_Test, readBuffer)
{
  // todo
}

TEST_F(Sx1262_Test, setDioIrqParams)
{
  // todo
}

TEST_F(Sx1262_Test, clearIrqStatus)
{
  expectHalWrite({
    +sx126x_commands_t::SX126X_CLR_IRQ_STATUS,
    static_cast<std::uint8_t>(+sx126x::IrqFlags::All >> 8),
    static_cast<std::uint8_t>(+sx126x::IrqFlags::All >> 0),
  });

  uut.clearIrqStatus(sx126x::IrqFlags::All);
}

TEST_F(Sx1262_Test, setRfFrequency)
{
  // todo
}

TEST_F(Sx1262_Test, calibrateImageMinMax)
{
  // todo
}

TEST_F(Sx1262_Test, setTxParams)
{
  // todo
}

TEST_F(Sx1262_Test, setModulationParams)
{
  // todo
}

TEST_F(Sx1262_Test, setPacketParams)
{
  // todo
}

TEST_F(Sx1262_Test, setBufferBaseAddress)
{
  // todo
}

TEST_F(Sx1262_Test, getPacketStatus)
{
  // todo
}

TEST_F(Sx1262_Test, getDeviceErrors)
{
  // todo
}

TEST_F(Sx1262_Test, clearDeviceErrors)
{
  // todo
}

TEST_F(Sx1262_Test, setFrequencyRaw)
{
  // todo
}

TEST_F(Sx1262_Test, fixPaClamping)
{
  // todo
}

TEST_F(Sx1262_Test, setFrequency)
{
  // todo
}

TEST_F(Sx1262_Test, setOutputPower)
{
  // todo
}
