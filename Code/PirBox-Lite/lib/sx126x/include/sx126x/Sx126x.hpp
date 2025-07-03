#pragma once

#include <sx126x/Types.hpp>

namespace sx126x {
class IHal;

enum class DeviceErrors : uint16_t;

class Sx126x
{
public:
  using CallBack = void (*)();

  explicit Sx126x(IHal& hal) noexcept;
  virtual ~Sx126x() = default;
  Sx126x(const Sx126x& other) = delete;
  Sx126x(Sx126x&& other) noexcept = delete;
  Sx126x& operator=(const Sx126x& other) = delete;
  Sx126x& operator=(Sx126x&& other) noexcept = delete;

  Result begin(CodingRate codingRate,
               uint8_t syncWord,
               uint16_t preambleLength,
               TcxoVoltage tcxoVoltage,
               bool useRegulatorLdo = false);

  Result reset(bool verify = true);
  Result sleep(bool warmStart = true);
  Result standby(Standby mode = Standby::Rc, bool wakeup = true);

  void setPacketReceivedOrSentAction(CallBack callback);
  void clearPacketReceivedOrSentAction();

  Result transmit(const uint8_t* data, size_t len);
  Result transmit(const char* str);
  Result transmit(const String& str);
  Result startTransmit(const uint8_t* data, size_t len);
  Result startTransmit(const char* str);
  Result startTransmit(const String& str);
  Result transmitDirect(uint32_t frequencyInHz = 0U);
  Result finishTransmit();

  Result receive(uint8_t* data, size_t len);
  Result receive(String& str, size_t len = 0U);
  Result startReceive(uint32_t timeout = g_rxTimeoutInf,
                      IrqFlags irqFlags = g_rxDefaultFlags,
                      IrqFlags irqMask = g_rxDefaultMask,
                      size_t len = 0U);
  Result readData(uint8_t* data, size_t len);
  Result readData(String& str, size_t len = 0U);

  Result stageMode(RadioMode mode, const RadioModeConfig& cfg);
  Result launchMode();

  Result setBandwidth(Bandwidth bandwidth);
  Result setSpreadingFactor(SpreadingFactor spreadingFactor);
  Result setCodingRate(CodingRate codingRate);
  Result setSyncWord(uint8_t syncWord);
  Result setCurrentLimit(float currentLimit);
  Result getCurrentLimit(float* value);
  Result setPreambleLength(size_t preambleLength);
  Result setCrc(bool enabled);
  Result setTcxo(TcxoVoltage voltage, uint32_t delayMs = 5000U);
  Result setDio2AsRfSwitch(bool enable = true);
  [[nodiscard]] float getDataRate() const;
  [[nodiscard]] float getRssi(bool packet = true);
  [[nodiscard]] float getSnr();
  [[nodiscard]] size_t getPacketLength(uint8_t* offset = nullptr);
  [[nodiscard]] Time getTimeOnAirInMs(uint8_t len);
  Result getIrqFlags(IrqFlags* irq);
  Result clearIrqFlags(IrqFlags irq = IrqFlags::All);
  Result implicitHeader(size_t len);
  Result explicitHeader();
  Result setRegulatorLdo();
  Result setRegulatorDcdc();
  Result forceLdro(bool enable);
  Result autoLdro();
  Result randomInt(uint32_t& value);
  Result invertIq(bool enable);
  Result setPaConfig(uint8_t paDutyCycle, uint8_t deviceSel, uint8_t hpMax = 0x07U, uint8_t paLut = 0x01U);
  Result calibrateImage(float frequency);

protected:
  Result setTx(uint32_t timeout);
  Result setRx(uint32_t timeout);
  Result writeBuffer(const uint8_t* data, uint8_t numBytes, uint8_t offset = 0x00U);
  Result readBuffer(uint8_t* data, uint8_t numBytes, uint8_t offset = 0x00U);
  Result setDioIrqParams(IrqFlags irqMask,
                         IrqFlags dio1Mask,
                         IrqFlags dio2Mask = IrqFlags::None,
                         IrqFlags dio3Mask = IrqFlags::None);
  Result clearIrqStatus(IrqFlags irq = IrqFlags::All);
  Result setRfFrequency(uint32_t frequencyInHz);
  Result calibrateImage(float frequencyMin, float frequencyMax);
  Result setTxParams(OutputPower outputPower, PaRampTime rampTime);
  Result setModulationParams(SpreadingFactor spreadingFactor,
                             Bandwidth bandwidth,
                             CodingRate codingRate,
                             bool ldrOptimize);
  Result setPacketParams(uint16_t preambleLen, bool crcOn, uint8_t payloadLen, PacketLengthMode hdrType, bool invertIq);
  Result setBufferBaseAddress(uint8_t txBaseAddress = 0x00U, uint8_t rxBaseAddress = 0x00U);
  Result getPacketStatus(PacketStatus* status);
  DeviceErrors getDeviceErrors();
  Result clearDeviceErrors();

  Result setFrequencyRaw(float frequencyInMhz);
  Result fixPaClamping();
  [[nodiscard]] float frequencyInMhz() const;

  // NOLINTBEGIN(*-non-private-member-variables-in-classes)
  IHal& m_hal;
  const char* m_chipType{nullptr};
  // NOLINTEND(*-non-private-member-variables-in-classes)

private:
  Result modSetup(TcxoVoltage tcxoVoltage, bool useRegulatorLdo);
  Result config();
  bool verifyChip();
  Result startReceiveCommon(uint32_t timeout = g_rxTimeoutInf,
                            IrqFlags irqFlags = g_rxDefaultFlags,
                            IrqFlags irqMask = g_rxDefaultMask);
  Result setHeaderType(PacketLengthMode headerType, size_t len = 0xFF);
  Result fixImplicitTimeout();

  float m_frequencyInMhz{0.0F};
  SpreadingFactor m_spreadingFactor{SpreadingFactor::_9};
  CodingRate m_codingRate{CodingRate::_4_7};
  bool m_ldrOptimize{false};
  bool m_crcEnabled{true};
  PacketLengthMode m_headerType{PacketLengthMode::Explicit};
  uint16_t m_preambleLength{0U};
  Bandwidth m_bandwidth{Bandwidth::_500_0};
  bool m_ldroAuto{true};

  float m_dataRateMeasured{0.0F};

  uint32_t m_tcxoDelayMs{0U};
  OutputPower m_outputPower{OutputPower::_10Dbm};

  size_t m_implicitLen{0xFFU};
  bool m_invertIqEnabled{false};
  uint32_t m_rxTimeout{0U};

  RadioMode m_stagedMode{RadioMode::None};
};
} // namespace sx126x
