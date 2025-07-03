#pragma once

#include <SPI.h>

#include <sx126x/IHal.hpp>

namespace sx126x {
class ArduinoHal final : public IHal
{
public:
  ArduinoHal(uint8_t chipSelectPin, uint8_t irqPin, uint8_t resetPin, uint8_t busyPin) noexcept;
  ArduinoHal(uint8_t chipSelectPin,
             uint8_t irqPin,
             uint8_t resetPin,
             uint8_t busyPin,
             SPIClass& spi,
             const SPISettings& spiSettings) noexcept;
  ~ArduinoHal() override = default;
  ArduinoHal(const ArduinoHal& other) = delete;
  ArduinoHal(ArduinoHal&& other) noexcept = delete;
  ArduinoHal& operator=(const ArduinoHal& other) = delete;
  ArduinoHal& operator=(ArduinoHal&& other) noexcept = delete;

  void init() override;
  void term() override;

  void pinMode(uint8_t pin, uint8_t mode) const override;
  void digitalWrite(uint8_t pin, uint8_t value) const override;
  [[nodiscard]] int8_t digitalRead(uint8_t pin) const override;

  void sleepMs(Time milliseconds) const override;
  [[nodiscard]] Time milliseconds() const override;
  void yield() const override;

  void spiBegin() const override;
  void spiBeginTransaction() const override;
  void spiTransfer(const uint8_t* dataOut, size_t length, uint8_t* dataIn) const override;
  void spiEndTransaction() const override;
  void spiEnd() const override;

  [[nodiscard]] bool waitForRadio() const override;

  [[nodiscard]] uint8_t chipSelectPin() const override;
  [[nodiscard]] uint8_t irqPin() const override;
  [[nodiscard]] uint8_t resetPin() const override;
  [[nodiscard]] uint8_t busyPin() const override;

  [[nodiscard]] uint8_t gpioModeInput() const override;
  [[nodiscard]] uint8_t gpioModeOutput() const override;
  [[nodiscard]] int8_t gpioLevelLow() const override;
  [[nodiscard]] int8_t gpioLevelHigh() const override;

  [[nodiscard]] uint8_t interruptRising() const override;
  [[nodiscard]] uint8_t interruptFalling() const override;

  [[nodiscard]] uint32_t pinToInterrupt(uint32_t pin) const override;
  void attachInterrupt(uint32_t interruptNum, CallBack interruptCallback, uint32_t mode) const override;
  void detachInterrupt(uint32_t interruptNum) const override;

  sx126x_hal_status_t write(const uint8_t* command,
                            uint16_t commandLength,
                            const uint8_t* data,
                            uint16_t dataLength,
                            bool waitUntilReady) const override;
  sx126x_hal_status_t read(const uint8_t* command,
                           uint16_t commandLength,
                           uint8_t* data,
                           uint16_t dataLength,
                           bool waitUntilReady) const override;
  [[nodiscard]] sx126x_hal_status_t reset() const override;
  [[nodiscard]] sx126x_hal_status_t wakeup() const override;

private:
  [[nodiscard]] sx126x_hal_status_t setStandby() const;

  uint8_t m_chipSelectPin;
  uint8_t m_irqPin;
  uint8_t m_resetPin;
  uint8_t m_busyPin;
  bool m_initInterface;
  SPIClass& m_spi;
  SPISettings m_spiSettings;
};
} // namespace sx126x
