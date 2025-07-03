#pragma once

#include <chrono>

#include <gmock/gmock.h>

#include <sx126x/IHal.hpp>

namespace sx126x {

class MockHal : public IHal
{
public:
  MOCK_METHOD(void, init, (), (override));
  MOCK_METHOD(void, term, (), (override));

  MOCK_METHOD(void, pinMode, (uint8_t pin, uint8_t mode), (const, override));
  MOCK_METHOD(void, digitalWrite, (uint8_t pin, uint8_t value), (const, override));
  MOCK_METHOD(int8_t, digitalRead, (uint8_t pin), (const, override));

  MOCK_METHOD(void, sleepMs, (Time milliseconds), (const, override));
  MOCK_METHOD(Time, milliseconds, (), (const, override));
  MOCK_METHOD(void, yield, (), (const, override));

  MOCK_METHOD(void, spiBegin, (), (const, override));
  MOCK_METHOD(void, spiBeginTransaction, (), (const, override));
  MOCK_METHOD(void, spiTransfer, (const uint8_t* out, size_t len, uint8_t* in), (const, override));
  MOCK_METHOD(void, spiEndTransaction, (), (const, override));
  MOCK_METHOD(void, spiEnd, (), (const, override));

  MOCK_METHOD(bool, waitForRadio, (), (const, override));

  MOCK_METHOD(uint8_t, chipSelectPin, (), (const, override));
  MOCK_METHOD(uint8_t, irqPin, (), (const, override));
  MOCK_METHOD(uint8_t, resetPin, (), (const, override));
  MOCK_METHOD(uint8_t, busyPin, (), (const, override));

  MOCK_METHOD(uint8_t, gpioModeInput, (), (const, override));
  MOCK_METHOD(uint8_t, gpioModeOutput, (), (const, override));
  MOCK_METHOD(int8_t, gpioLevelLow, (), (const, override));
  MOCK_METHOD(int8_t, gpioLevelHigh, (), (const, override));

  MOCK_METHOD(uint8_t, interruptRising, (), (const, override));
  MOCK_METHOD(uint8_t, interruptFalling, (), (const, override));

  MOCK_METHOD(uint32_t, pinToInterrupt, (uint32_t pin), (const, override));
  MOCK_METHOD(void,
              attachInterrupt,
              (uint32_t interruptNum, CallBack interruptCallback, uint32_t mode),
              (const, override));
  MOCK_METHOD(void, detachInterrupt, (uint32_t interruptNum), (const, override));

  MOCK_METHOD(sx126x_hal_status_t,
              write,
              (const uint8_t* command,
               const uint16_t command_length,
               const uint8_t* data,
               const uint16_t data_length,
               bool waitUntilReady),
              (const, override));
  MOCK_METHOD(sx126x_hal_status_t,
              read,
              (const uint8_t* command,
               const uint16_t command_length,
               uint8_t* data,
               const uint16_t data_length,
               bool waitUntilReady),
              (const, override));
  MOCK_METHOD(sx126x_hal_status_t, reset, (), (const, override));
  MOCK_METHOD(sx126x_hal_status_t, wakeup, (), (const, override));
};

} // namespace sx126x
