#include <arduino_hal/ArduinoHal.hpp>

#include <Arduino.h>
#include <SPI.h>

namespace driver {
#include <sx126x.h>
#include <sx126x_hal.h>
} // namespace driver

// ReSharper disable CppRedundantQualifier

namespace sx126x {

namespace {

constexpr Time g_spiTimeoutInMillis{1000U};
constexpr uint8_t g_setStandyByCommand{0x80U};

class Transfer
{
public:
  Transfer(const Transfer& other) = delete;
  Transfer(Transfer&& other) noexcept = delete;
  Transfer& operator=(const Transfer& other) = delete;
  Transfer& operator=(Transfer&& other) noexcept = delete;

  explicit Transfer(const IHal& hal)
    : m_hal{hal}
  {
    m_hal.spiBeginTransaction();
    m_hal.digitalWrite(m_hal.chipSelectPin(), m_hal.gpioLevelLow());
  }

  ~Transfer()
  {
    m_hal.digitalWrite(m_hal.chipSelectPin(), m_hal.gpioLevelHigh());
    m_hal.spiEndTransaction();
  }

  // ReSharper disable once CppMemberFunctionMayBeConst
  void operator()(const uint8_t* dataOut, const size_t length, uint8_t* const dataIn = nullptr)
  {
    m_hal.spiTransfer(dataOut, length, dataIn);
  }

private:
  const IHal& m_hal;
};

} // namespace

ArduinoHal::ArduinoHal(const uint8_t chipSelectPin,
                       const uint8_t irqPin,
                       const uint8_t resetPin,
                       const uint8_t busyPin) noexcept
  // NOLINTNEXTLINE(*-signed-bitwise)
  : ArduinoHal{chipSelectPin, irqPin, resetPin, busyPin, SPI, SPISettings{2000000, MSBFIRST, SPI_MODE0}}
{
  m_initInterface = true;
}

ArduinoHal::ArduinoHal(const uint8_t chipSelectPin,
                       const uint8_t irqPin,
                       const uint8_t resetPin,
                       const uint8_t busyPin,
                       SPIClass& spi,
                       const SPISettings& spiSettings) noexcept
  : m_chipSelectPin{chipSelectPin}
  , m_irqPin{irqPin}
  , m_resetPin{resetPin}
  , m_busyPin{busyPin}
  , m_initInterface{false}
  , m_spi{spi}
  , m_spiSettings{spiSettings}
{
}

void
ArduinoHal::init()
{
  if (m_initInterface) {
    spiBegin();
  }
}

void
ArduinoHal::term()
{
  if (m_initInterface) {
    spiEnd();
  }
}

void
ArduinoHal::pinMode(const uint8_t pin, const uint8_t mode) const
{
  ::pinMode(pin, mode);
}

void
ArduinoHal::digitalWrite(const uint8_t pin, const uint8_t value) const
{
  ::digitalWrite(pin, value);
}

int8_t
ArduinoHal::digitalRead(const uint8_t pin) const
{
  return ::digitalRead(pin);
}

void
ArduinoHal::sleepMs(const Time milliseconds) const
{
  delay(milliseconds);
}

Time
ArduinoHal::milliseconds() const
{
  return millis();
}

void
ArduinoHal::yield() const
{
  ::yield();
}

void
ArduinoHal::spiBegin() const
{
  m_spi.begin();
}

void
ArduinoHal::spiBeginTransaction() const
{
  m_spi.beginTransaction(m_spiSettings);
}

void
ArduinoHal::spiTransfer(const uint8_t* dataOut, const size_t length, uint8_t* const dataIn) const
{
  const auto transfer = [&](const size_t index) {
    return m_spi.transfer(dataOut != nullptr ? dataOut[index] : SX126X_NOP);
  };

  for (size_t index{0U}; index < length; ++index) {
    if (dataIn != nullptr) {
      dataIn[index] = transfer(index);
    } else {
      transfer(index);
    }
  }
}

void
ArduinoHal::spiEndTransaction() const
{
  m_spi.endTransaction();
}

void
ArduinoHal::spiEnd() const
{
  m_spi.end();
}

bool
ArduinoHal::waitForRadio() const
{
  delayMicroseconds(1);
  const Time start = milliseconds();
  while (digitalRead(busyPin()) == gpioLevelHigh()) {
    if (milliseconds() - start >= g_spiTimeoutInMillis) {
      return false;
    }
    yield();
  }
  return true;
}

uint8_t
ArduinoHal::chipSelectPin() const
{
  return m_chipSelectPin;
}

uint8_t
ArduinoHal::irqPin() const
{
  return m_irqPin;
}

uint8_t
ArduinoHal::resetPin() const
{
  return m_resetPin;
}

uint8_t
ArduinoHal::busyPin() const
{
  return m_busyPin;
}

uint8_t
ArduinoHal::gpioModeInput() const
{
  return INPUT;
}

uint8_t
ArduinoHal::gpioModeOutput() const
{
  return OUTPUT;
}

int8_t
ArduinoHal::gpioLevelLow() const
{
  return LOW;
}

int8_t
ArduinoHal::gpioLevelHigh() const
{
  return HIGH;
}

uint8_t
ArduinoHal::interruptRising() const
{
  return RISING;
}

uint8_t
ArduinoHal::interruptFalling() const
{
  return FALLING;
}

uint32_t
ArduinoHal::pinToInterrupt(const uint32_t pin) const
{
  return digitalPinToInterrupt(pin);
}

void
ArduinoHal::attachInterrupt(const uint32_t interruptNum, const CallBack interruptCallback, const uint32_t mode) const
{
  ::attachInterrupt(interruptNum, interruptCallback, mode);
}

void
ArduinoHal::detachInterrupt(const uint32_t interruptNum) const
{
  ::detachInterrupt(interruptNum);
}

sx126x_hal_status_t
ArduinoHal::write(const uint8_t* command,
                  const uint16_t commandLength,
                  const uint8_t* data,
                  const uint16_t dataLength,
                  const bool waitUntilReady) const
{
  if (waitUntilReady and not waitForRadio()) {
    return static_cast<sx126x_hal_status_t>(Result::SpiCmdTimeout);
  }

  Transfer transfer{*this};

  transfer(command, commandLength);
  if (data != nullptr and dataLength > 0U) {
    transfer(data, dataLength);
  }

  if (waitUntilReady and not waitForRadio()) {
    return static_cast<sx126x_hal_status_t>(Result::SpiCmdTimeout);
  }

  return sx126x_hal_status_t::SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t
ArduinoHal::read(const uint8_t* command,
                 const uint16_t commandLength,
                 uint8_t* data,
                 const uint16_t dataLength,
                 const bool waitUntilReady) const
{
  if (waitUntilReady and not waitForRadio()) {
    return static_cast<sx126x_hal_status_t>(Result::SpiCmdTimeout);
  }

  Transfer transfer{*this};

  transfer(command, commandLength);
  transfer(nullptr, dataLength, data);

  if (waitUntilReady and not waitForRadio()) {
    return static_cast<sx126x_hal_status_t>(Result::SpiCmdTimeout);
  }

  return sx126x_hal_status_t::SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t
ArduinoHal::reset() const
{
  pinMode(resetPin(), gpioModeOutput());
  digitalWrite(resetPin(), gpioLevelLow());
  sleepMs(1U);
  digitalWrite(resetPin(), gpioLevelHigh());

  // set mode to standby - SX126x often refuses first few commands after reset
  const auto start = milliseconds();
  while (true) {
    const sx126x_hal_status_t state = setStandby();
    if (state == sx126x_hal_status_t::SX126X_HAL_STATUS_OK) {
      return state;
    }

    if (milliseconds() - start >= 1000U) {
      // timed out, possibly incorrect wiring
      return state;
    }

    // wait a bit to not spam the module
    sleepMs(10U);
  }
}

sx126x_hal_status_t
ArduinoHal::wakeup() const
{
  constexpr uint8_t buf[] = {
    SX126X_NOP,
  };

  return write(buf, sizeof(buf), nullptr, 0U, false);
}

sx126x_hal_status_t
ArduinoHal::setStandby() const
{
  constexpr uint8_t buf[] = {
    g_setStandyByCommand,
    driver::sx126x_standby_cfgs_t::SX126X_STANDBY_CFG_RC,
  };

  return write(buf, sizeof(buf), nullptr, 0U, true);
}

} // namespace sx126x
