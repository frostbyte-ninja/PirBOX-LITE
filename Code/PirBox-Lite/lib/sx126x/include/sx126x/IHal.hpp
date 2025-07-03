#pragma once

#include <stddef.h>
#include <stdint.h>

#include <sx126x/Types.hpp>

#include <sx126x_hal.h>

namespace sx126x {
class IHal
{
public:
  using CallBack = void (*)();

  virtual ~IHal() = default;

  virtual void init() = 0;
  virtual void term() = 0;

  virtual void pinMode(uint8_t pin, uint8_t mode) const = 0;
  virtual void digitalWrite(uint8_t pin, uint8_t value) const = 0;
  [[nodiscard]] virtual int8_t digitalRead(uint8_t pin) const = 0;

  virtual void sleepMs(Time milliseconds) const = 0;
  [[nodiscard]] virtual Time milliseconds() const = 0;
  virtual void yield() const = 0;

  virtual void spiBegin() const = 0;
  virtual void spiBeginTransaction() const = 0;
  virtual void spiTransfer(const uint8_t* dataOut, size_t length, uint8_t* dataIn) const = 0;
  virtual void spiEndTransaction() const = 0;
  virtual void spiEnd() const = 0;

  virtual bool waitForRadio() const = 0; // NOLINT(*-use-nodiscard)

  [[nodiscard]] virtual uint8_t chipSelectPin() const = 0;
  [[nodiscard]] virtual uint8_t resetPin() const = 0;
  [[nodiscard]] virtual uint8_t busyPin() const = 0;
  [[nodiscard]] virtual uint8_t irqPin() const = 0;

  [[nodiscard]] virtual uint8_t gpioModeInput() const = 0;
  [[nodiscard]] virtual uint8_t gpioModeOutput() const = 0;
  [[nodiscard]] virtual int8_t gpioLevelLow() const = 0;
  [[nodiscard]] virtual int8_t gpioLevelHigh() const = 0;

  [[nodiscard]] virtual uint8_t interruptRising() const = 0;
  [[nodiscard]] virtual uint8_t interruptFalling() const = 0;

  [[nodiscard]] virtual uint32_t pinToInterrupt(uint32_t pin) const = 0;
  virtual void attachInterrupt(uint32_t interruptNum, CallBack interruptCallback, uint32_t mode) const = 0;
  virtual void detachInterrupt(uint32_t interruptNum) const = 0;

  virtual sx126x_hal_status_t write(const uint8_t* command,
                                    uint16_t commandLength,
                                    const uint8_t* data,
                                    uint16_t dataLength,
                                    bool waitUntilReady) const = 0;
  virtual sx126x_hal_status_t read(const uint8_t* command,
                                   uint16_t commandLength,
                                   uint8_t* data,
                                   uint16_t dataLength,
                                   bool waitUntilReady) const = 0;
  [[nodiscard]] virtual sx126x_hal_status_t reset() const = 0;
  [[nodiscard]] virtual sx126x_hal_status_t wakeup() const = 0;

protected:
  IHal() = default;
};
} // namespace sx126x
