#include <sx126x/Sx126x.hpp>

#include <etl/limits.h>
#include <etl/memory.h>

namespace driver {
#include <sx126x.h>
#include <sx126x_regs.h>
} // namespace driver

#include <sx126x/Assert.hpp>
#include <sx126x/IHal.hpp>

#include "InternalTypes.hpp"

// ReSharper disable CppMemberFunctionMayBeConst

#ifdef max
#undef max
#endif

namespace sx126x {

Sx126x::Sx126x(IHal& hal) noexcept
  : m_hal{hal}
{
}

Result
Sx126x::begin(const CodingRate codingRate,
              const uint8_t syncWord,
              const uint16_t preambleLength,
              const TcxoVoltage tcxoVoltage,
              const bool useRegulatorLdo)
{
  m_preambleLength = preambleLength;

  // set module properties and perform initial setup
  ASSERT(modSetup(tcxoVoltage, useRegulatorLdo));

  // configure publicly accessible settings
  ASSERT(setCodingRate(codingRate));
  ASSERT(setSyncWord(syncWord));
  ASSERT(setPreambleLength(preambleLength));

  // set publicly accessible settings that are not a part of begin method
  ASSERT(setCurrentLimit(60.0F));
  ASSERT(setDio2AsRfSwitch(true));
  ASSERT(setCrc(true));
  ASSERT(invertIq(false));

  return Result::Ok;
}

Result
Sx126x::reset(const bool verify)
{
  // run the reset sequence
  m_hal.pinMode(m_hal.resetPin(), m_hal.gpioModeOutput());
  m_hal.digitalWrite(m_hal.resetPin(), m_hal.gpioLevelLow());
  m_hal.sleepMs(1);
  m_hal.digitalWrite(m_hal.resetPin(), m_hal.gpioLevelHigh());

  // return immediately when verification is disabled
  if (not verify) {
    return Result::Ok;
  }

  // set mode to standby - SX126x often refuses first few commands after reset
  const Time start = m_hal.milliseconds();
  while (true) {
    // try to set mode to standby
    const Result result = standby();
    if (result == Result::Ok) {
      // standby command successful
      return Result::Ok;
    }

    // standby command failed, check timeout and try again
    if (m_hal.milliseconds() - start >= 1000) {
      // timed out, possibly incorrect wiring
      return result;
    }

    // wait a bit to not spam the module
    m_hal.sleepMs(10);
  }
}

Result
Sx126x::sleep(const bool warmStart)
{
  const auto result = driver::sx126x_set_sleep(&m_hal,
                                               warmStart ? driver::sx126x_sleep_cfgs_t::SX126X_SLEEP_CFG_WARM_START
                                                         : driver::sx126x_sleep_cfgs_t::SX126X_SLEEP_CFG_COLD_START);

  if (result == driver::sx126x_status_t::SX126X_STATUS_OK) {
    m_hal.sleepMs(1);
  }

  return toResult(result);
}

Result
Sx126x::standby(const Standby mode, const bool wakeup)
{
  if (wakeup) {
    driver::sx126x_wakeup(&m_hal);
  }

  return toResult(driver::sx126x_set_standby(&m_hal, static_cast<driver::sx126x_standby_cfg_t>(mode)));
}

void
Sx126x::setPacketReceivedOrSentAction(const CallBack callback)
{
  m_hal.attachInterrupt(m_hal.pinToInterrupt(m_hal.irqPin()), callback, m_hal.interruptRising());
}

void
Sx126x::clearPacketReceivedOrSentAction()
{
  m_hal.detachInterrupt(m_hal.pinToInterrupt(m_hal.irqPin()));
}

Result
Sx126x::transmit(const uint8_t* data, const size_t len)
{
  ASSERT(standby());

  if (len > g_maxPacketLength) {
    return Result::PacketTooLong;
  }

  // calculate timeout in ms (5ms + 500 % of expected time-on-air)
  // ReSharper disable once CppRedundantParentheses
  const auto timeout = 5U + (getTimeOnAirInMs(len) * 5U);

  ASSERT(startTransmit(data, len));

  // wait for packet transmission or timeout
  const auto start = m_hal.milliseconds();
  do {
    m_hal.yield();

    // check timeout
    if (m_hal.milliseconds() - start > timeout) {
      finishTransmit();
      return Result::TxTimeout;
    }

  } while (m_hal.digitalRead(m_hal.irqPin()) == m_hal.gpioLevelLow());

  const Time elapsed = m_hal.milliseconds() - start;
  // ReSharper disable once CppRedundantParentheses
  m_dataRateMeasured = (static_cast<float>(len) * 8.0F) / (static_cast<float>(elapsed) / 1000.0F);

  return finishTransmit();
}

Result
Sx126x::transmit(const char* str)
{
  return transmit(reinterpret_cast<const uint8_t*>(str), strlen(str));
}

Result
Sx126x::transmit(const String& str)
{
  return transmit(str.c_str());
}

Result
Sx126x::startTransmit(const uint8_t* data, const size_t len)
{
  const RadioModeConfig cfg{.transmit = {
                              .data = data,
                              .len = len,
                            }};

  ASSERT(stageMode(RadioMode::Tx, cfg));
  ASSERT(launchMode());

  return Result::Ok;
}

Result
Sx126x::startTransmit(const char* str)
{
  return startTransmit(reinterpret_cast<const uint8_t*>(str), strlen(str));
}

Result
Sx126x::startTransmit(const String& str)
{
  return startTransmit(str.c_str());
}

Result
Sx126x::transmitDirect(const uint32_t frequencyInHz)
{
  // user requested to start transmitting immediately (required for RTTY)
  if (frequencyInHz != 0U) {
    ASSERT(setRfFrequency(frequencyInHz));
  }

  // direct mode activation intentionally skipped here, as it seems to lead to much worse results
  return toResult(driver::sx126x_set_tx_cw(&m_hal));
}

Result
Sx126x::finishTransmit()
{
  ASSERT(clearIrqStatus());

  // set mode to standby to disable transmitter/RF switch
  return standby();
}

Result
Sx126x::receive(uint8_t* const data, const size_t len)
{
  // set mode to standby
  ASSERT(standby());

  Time timeout = 0U;

  // calculate timeout (100 LoRa symbols, the default for SX127x series)
  const float symbolLength = static_cast<float>(1U << +m_spreadingFactor) / static_cast<float>(toKhz(m_bandwidth));
  timeout = static_cast<Time>(symbolLength * 100.0F);

  // start reception
  // ReSharper disable once CppRedundantParentheses
  const auto timeoutValue = static_cast<uint32_t>((static_cast<float>(timeout) * 1000.0F) / 15.625F);
  ASSERT(startReceive(timeoutValue));

  // wait for packet reception or timeout
  bool softTimeout = false;
  const Time start = m_hal.milliseconds();
  while (m_hal.digitalRead(m_hal.irqPin()) == m_hal.gpioLevelLow()) {
    m_hal.yield();
    // safety check, the timeout should be done by the radio
    if (m_hal.milliseconds() - start > timeout) {
      softTimeout = true;
      break;
    }
  }

  // if it was a timeout, this will return an error code
  if (const auto state = standby(); state != Result::Ok and state != Result::SpiCmdTimeout) {
    return state;
  }

  // check whether this was a timeout or not
  IrqFlags irqFlags{};
  ASSERT(getIrqFlags(&irqFlags));

  if (hasFlag(irqFlags, IrqFlags::Timeout) or softTimeout) {
    standby();
    fixImplicitTimeout();
    clearIrqStatus();
    return Result::RxTimeout;
  }

  // fix timeout in implicit LoRa mode
  if (m_headerType == PacketLengthMode::Implicit) {
    ASSERT(fixImplicitTimeout());
  }

  // read the received data
  return readData(data, len);
}

// ReSharper disable once CppDFAConstantFunctionResult
Result
Sx126x::receive(String& str, const size_t len)
{
  // user can override the length of data to read
  size_t length = len;

  // build a temporary buffer
  const size_t bufferSize = (length == 0 ? g_maxPacketLength : length) + 1;
  const etl::unique_ptr data{new uint8_t[bufferSize]};

  ASSERT_PTR(data.get());

  // any of the following leads to at least some data being available
  // let's leave the decision of whether to keep it or not up to the user
  if (const auto state = receive(data.get(), length); state == Result::Ok or state == Result::CrcMismatch) {
    // read the number of actually received bytes (for unknown packets)
    if (len == 0) {
      length = getPacketLength();
    }

    // add null terminator
    data[length] = '0';

    // initialize Arduino String class
    str = reinterpret_cast<char*>(data.get());
  }

  return Result::Ok;
}

Result
Sx126x::startReceive(const uint32_t timeout, const IrqFlags irqFlags, const IrqFlags irqMask, const size_t len)
{
  const RadioModeConfig cfg = {.receive = {
                                 .timeout = timeout,
                                 .irqFlags = irqFlags,
                                 .irqMask = irqMask,
                                 .len = len,
                               }};

  ASSERT(stageMode(RadioMode::Rx, cfg));
  ASSERT(launchMode());

  return Result::Ok;
}

Result
Sx126x::readData(uint8_t* const data, const size_t len)
{
  // this method may get called from receive() after Rx timeout
  // if that's the case, the first call will return "SPI command timeout error"
  // check the IRQ to be sure this really originated from timeout event
  IrqFlags irqFlags{};
  ASSERT(getIrqFlags(&irqFlags));

  if (hasFlag(irqFlags, IrqFlags::Timeout)) {
    // this is definitely Rx timeout
    return Result::RxTimeout;
  }

  // check integrity CRC
  auto crcState = Result::Ok;
  // Report CRC mismatch when there's a payload CRC error, or a header error and no valid header (to avoid false alarm
  // from previous packet)
  if (hasFlag(irqFlags, IrqFlags::CrcError) or
      (hasFlag(irqFlags, IrqFlags::HeaderError) and not hasFlag(irqFlags, IrqFlags::HeaderValid))) {
    crcState = Result::CrcMismatch;
  }

  // get packet length and Rx buffer offset
  uint8_t offset = 0;
  size_t length = getPacketLength(&offset);
  if (len != 0 and len < length) {
    // user requested less data than we got, only return what was requested
    length = len;
  }

  // read packet data starting at offset
  ASSERT(readBuffer(data, length, offset));

  // clear interrupt flags
  const auto state = clearIrqStatus();

  // check if CRC failed - this is done after reading data to give user the option to keep them
  ASSERT(crcState);

  ASSERT(state);

  return Result::Ok;
}

Result
Sx126x::readData(String& str, const size_t len)
{
  // read the number of actually received bytes
  size_t length = getPacketLength();

  if (len < length and len != 0) {
    // user requested less bytes than were received, this is allowed (but frowned upon)
    // requests for more data than were received will only return the number of actually received bytes (unlike
    // PhysicalLayer::receive())
    length = len;
  }

  // build a temporary buffer
  const etl::unique_ptr data{new uint8_t[length + 1]};

  ASSERT_PTR(data.get());

  // read the received data
  const auto state = readData(data.get(), length);

  // any of the following leads to at least some data being available
  // let's leave the decision of whether to keep it or not up to the user
  if (state == Result::Ok or state == Result::CrcMismatch) {
    // add null terminator
    data[length] = '0';

    // initialize Arduino String class
    str = reinterpret_cast<char*>(data.get());
  }

  return state;
}

Result
Sx126x::stageMode(const RadioMode mode, const RadioModeConfig& cfg) // NOLINT(*-function-cognitive-complexity)
{
  switch (mode) {
    case RadioMode::Rx: {
      // in implicit header mode, use the provided length if it is nonzero
      // otherwise we trust the user has previously set the payload length manually
      if (m_headerType == PacketLengthMode::Implicit and cfg.receive.len != 0) {
        m_implicitLen = cfg.receive.len;
      }

      ASSERT(startReceiveCommon(cfg.receive.timeout, cfg.receive.irqFlags, cfg.receive.irqMask));

      // if max(uint32_t) is used, revert to RxContinuous
      m_rxTimeout = cfg.receive.timeout == etl::numeric_limits<decltype(cfg.receive.timeout)>::max()
                      ? SX126X_RX_CONTINUOUS
                      : cfg.receive.timeout;
    } break;

    case RadioMode::Tx: {
      // check packet length
      if (cfg.transmit.len > g_maxPacketLength) {
        return Result::PacketTooLong;
      }

      ASSERT(setPacketParams(m_preambleLength, m_crcEnabled, cfg.transmit.len, m_headerType, m_invertIqEnabled));
      ASSERT(setDioIrqParams(IrqFlags::TxDone | IrqFlags::Timeout, IrqFlags::TxDone));
      ASSERT(setBufferBaseAddress());
      ASSERT(writeBuffer(cfg.transmit.data, cfg.transmit.len));
      ASSERT(clearIrqStatus());
    } break;

    default:
      return Result::Unsupported;
  }

  m_stagedMode = mode;

  return Result::Ok;
}

Result
Sx126x::launchMode()
{
  auto state{Result::Ok};

  switch (m_stagedMode) {
    case RadioMode::Rx: {
      state = setRx(m_rxTimeout);
    } break;
    case RadioMode::Tx: {
      ASSERT(setTx(g_txNoTimeout));
      // ReSharper disable once CppExpressionWithoutSideEffects
      while (not m_hal.waitForRadio()) {
      }
    } break;
    default: {
      return Result::Unsupported;
    }
  }

  m_stagedMode = RadioMode::None;

  return state;
}

Result
Sx126x::setBandwidth(const Bandwidth bandwidth)
{
  m_bandwidth = bandwidth;
  return setModulationParams(m_spreadingFactor, m_bandwidth, m_codingRate, m_ldrOptimize);
}

Result
Sx126x::setSpreadingFactor(const SpreadingFactor spreadingFactor)
{
  m_spreadingFactor = spreadingFactor;
  return setModulationParams(m_spreadingFactor, m_bandwidth, m_codingRate, m_ldrOptimize);
}

Result
Sx126x::setCodingRate(const CodingRate codingRate)
{
  m_codingRate = codingRate;
  return setModulationParams(m_spreadingFactor, m_bandwidth, m_codingRate, m_ldrOptimize);
}

Result
Sx126x::setSyncWord(const uint8_t syncWord)
{
  return toResult(driver::sx126x_set_lora_sync_word(&m_hal, syncWord));
}

Result
Sx126x::setCurrentLimit(const float currentLimit)
{
  // check allowed range
  ASSERT_RANGE(currentLimit, 0.0F, 140.0F, Result::InvalidCurrentLimit);

  // calculate raw value
  const auto ocp = static_cast<uint8_t>(currentLimit / 2.5F);

  // update register
  return toResult(driver::sx126x_set_ocp_value(&m_hal, ocp));
}

Result
Sx126x::getCurrentLimit(float* const value)
{
  uint8_t ocp{};
  const auto result = driver::sx126x_read_register(&m_hal, SX126X_REG_OCP, &ocp, sizeof(ocp));
  if (result == driver::sx126x_status_t::SX126X_STATUS_OK) {
    *value = static_cast<float>(ocp) * 2.5F;
  }
  return toResult(result);
}

Result
Sx126x::setPreambleLength(const size_t preambleLength)
{
  m_preambleLength = preambleLength;
  return setPacketParams(m_preambleLength, m_crcEnabled, m_implicitLen, m_headerType, m_invertIqEnabled);
}

Result
Sx126x::setCrc(const bool enabled)
{
  m_crcEnabled = enabled;

  return setPacketParams(m_preambleLength, m_crcEnabled, m_implicitLen, m_headerType, m_invertIqEnabled);
}

Result
Sx126x::setTcxo(const TcxoVoltage voltage, const uint32_t delayMs)
{
  // set mode to standby
  standby();

  // check RADIOLIB_SX126X_XOSC_START_ERR flag and clear it
  if (hasError(getDeviceErrors(), DeviceErrors::XoscStart)) {
    clearDeviceErrors();
  }

  // check 0 V disable
  if (voltage == TcxoVoltage::_0V) {
    return reset(true);
  }

  m_tcxoDelayMs = delayMs;

  // enable TCXO control on DIO3
  const uint32_t delayConverted = static_cast<float>(delayMs) / 15.625F; // NOLINT(*-narrowing-conversions)
  return toResult(driver::sx126x_set_dio3_as_tcxo_ctrl(
    &m_hal, static_cast<driver::sx126x_tcxo_ctrl_voltages_e>(voltage), delayConverted));
}

Result
Sx126x::setDio2AsRfSwitch(const bool enable)
{
  return toResult(driver::sx126x_set_dio2_as_rf_sw_ctrl(&m_hal, enable));
}

float
Sx126x::getDataRate() const
{
  return m_dataRateMeasured;
}

float
Sx126x::getRssi(const bool packet)
{
  int16_t rssiInDbm{0U};
  if (packet) {
    // get last packet RSSI from packet status
    PacketStatus packetStatus{};
    if (getPacketStatus(&packetStatus) == Result::Ok) {
      rssiInDbm = packetStatus.rssiPktInDbm; // NOLINT(*-signed-char-misuse)
    }
  } else {
    // get instantaneous RSSI value
    int16_t rssiValue{0};
    if (driver::sx126x_get_rssi_inst(&m_hal, &rssiValue) == driver::sx126x_status_t::SX126X_STATUS_OK) {
      rssiInDbm = rssiValue;
    }
  }

  return static_cast<float>(rssiInDbm) / -2.0F;
}

float
Sx126x::getSnr()
{
  PacketStatus packetStatus{};
  if (getPacketStatus(&packetStatus) != Result::Ok) {
    return 0.0F;
  }

  return packetStatus.snrPktInDb;
}

size_t
Sx126x::getPacketLength(uint8_t* const offset)
{
  // in implicit mode, return the cached value
  if (m_headerType == PacketLengthMode::Implicit) {
    return m_implicitLen;
  }

  driver::sx126x_rx_buffer_status_t bufferStatus{
    .pld_len_in_bytes = 0U,
    .buffer_start_pointer = 0U,
  };

  driver::sx126x_get_rx_buffer_status(&m_hal, &bufferStatus);

  if (offset != nullptr) {
    *offset = bufferStatus.buffer_start_pointer;
  }

  return bufferStatus.pld_len_in_bytes;
}

Time
Sx126x::getTimeOnAirInMs(const uint8_t len)
{
  const driver::sx126x_pkt_params_lora_t pktP{
    .preamble_len_in_symb = m_preambleLength,
    .header_type = static_cast<driver::sx126x_lora_pkt_len_modes_t>(m_headerType),
    .pld_len_in_bytes = len,
    .crc_is_on = m_crcEnabled,
    .invert_iq_is_on = m_invertIqEnabled,
  };

  const driver::sx126x_mod_params_lora_t modP{
    .sf = static_cast<driver::sx126x_lora_sf_t>(m_spreadingFactor),
    .bw = static_cast<driver::sx126x_lora_bw_t>(m_bandwidth),
    .cr = static_cast<driver::sx126x_lora_cr_t>(m_codingRate),
    .ldro = static_cast<uint8_t>(m_ldrOptimize),
  };

  return driver::sx126x_get_lora_time_on_air_in_ms(&pktP, &modP);
}

Result
Sx126x::getIrqFlags(IrqFlags* const irq)
{
  driver::sx126x_irq_mask_t status{};
  const auto result = driver::sx126x_get_irq_status(&m_hal, &status);
  if (result == driver::sx126x_status_t::SX126X_STATUS_OK) {
    *irq = static_cast<IrqFlags>(status);
  }
  return toResult(result);
}

Result
Sx126x::clearIrqFlags(const IrqFlags irq)
{
  return clearIrqStatus(irq);
}

Result
Sx126x::implicitHeader(const size_t len)
{
  return setHeaderType(PacketLengthMode::Implicit, len);
}

Result
Sx126x::explicitHeader()
{
  return setHeaderType(PacketLengthMode::Explicit);
}

Result
Sx126x::setRegulatorLdo()
{
  return toResult(driver::sx126x_set_reg_mode(&m_hal, driver::sx126x_reg_mod_t::SX126X_REG_MODE_LDO));
}

Result
Sx126x::setRegulatorDcdc()
{
  return toResult(driver::sx126x_set_reg_mode(&m_hal, driver::sx126x_reg_mod_t::SX126X_REG_MODE_DCDC));
}

Result
Sx126x::forceLdro(const bool enable)
{
  // update modulation parameters
  m_ldroAuto = false;
  m_ldrOptimize = enable;
  return setModulationParams(m_spreadingFactor, m_bandwidth, m_codingRate, m_ldrOptimize);
}

Result
Sx126x::autoLdro()
{
  m_ldroAuto = true;
  return Result::Ok;
}

Result
Sx126x::randomInt(uint32_t& value)
{
  return toResult(driver::sx126x_get_random_numbers(&m_hal, &value, 1U));
}

Result
Sx126x::invertIq(const bool enable)
{
  m_invertIqEnabled = enable;

  return setPacketParams(m_preambleLength, m_crcEnabled, m_implicitLen, m_headerType, m_invertIqEnabled);
}

Result
Sx126x::setPaConfig(const uint8_t paDutyCycle, const uint8_t deviceSel, const uint8_t hpMax, const uint8_t paLut)
{
  const driver::sx126x_pa_cfg_params_t params{
    .pa_duty_cycle = paDutyCycle,
    .hp_max = hpMax,
    .device_sel = deviceSel,
    .pa_lut = paLut,
  };

  return toResult(driver::sx126x_set_pa_cfg(&m_hal, &params));
}

Result
Sx126x::calibrateImage(const float frequency)
{
  uint8_t frequencyMin{};
  uint8_t frequencyMax{};

  // try to match the frequency ranges
  // ReSharper disable once CppTooWideScopeInitStatement
  const auto frequencyBand = static_cast<int>(frequency);
  if (frequencyBand >= 902 and frequencyBand <= 928) {
    frequencyMin = g_calibrateImage902Mhz1;
    frequencyMax = g_calibrateImage902Mhz2;
  } else if (frequencyBand >= 863 and frequencyBand <= 870) {
    frequencyMin = g_calibrateImage863Mhz1;
    frequencyMax = g_calibrateImage863Mhz2;
  } else if (frequencyBand >= 779 and frequencyBand <= 787) {
    frequencyMin = g_calibrateImage779Mhz1;
    frequencyMax = g_calibrateImage779Mhz2;
  } else if (frequencyBand >= 470 and frequencyBand <= 510) {
    frequencyMin = g_calibrateImage470Mhz1;
    frequencyMax = g_calibrateImage470Mhz2;
  } else if (frequencyBand >= 430 and frequencyBand <= 440) {
    frequencyMin = g_calibrateImage430Mhz1;
    frequencyMax = g_calibrateImage430Mhz2;
  } else {
    // if nothing matched, try custom calibration - the may or may not work
    frequencyMin = calibrationFrequencyMin(frequency - 4.0F);
    frequencyMax = calibrationFrequencyMax(frequency + 4.0F);
  }

  return calibrateImage(frequencyMin, frequencyMax);
}

Result
Sx126x::setTx(const uint32_t timeout)
{
  return toResult(driver::sx126x_set_tx(&m_hal, timeout));
}

Result
Sx126x::setRx(const uint32_t timeout)
{
  return toResult(driver::sx126x_set_rx_with_timeout_in_rtc_step(&m_hal, timeout));
}

Result
Sx126x::writeBuffer(const uint8_t* data, const uint8_t numBytes, const uint8_t offset)
{
  return toResult(driver::sx126x_write_buffer(&m_hal, offset, data, numBytes));
}

Result
Sx126x::readBuffer(uint8_t* const data, const uint8_t numBytes, const uint8_t offset)
{
  return toResult(driver::sx126x_read_buffer(&m_hal, offset, data, numBytes));
}

Result
Sx126x::setDioIrqParams(const IrqFlags irqMask,
                        const IrqFlags dio1Mask,
                        const IrqFlags dio2Mask,
                        const IrqFlags dio3Mask)
{
  return toResult(driver::sx126x_set_dio_irq_params(&m_hal,
                                                    static_cast<uint16_t>(irqMask),
                                                    static_cast<uint16_t>(dio1Mask),
                                                    static_cast<uint16_t>(dio2Mask),
                                                    static_cast<uint16_t>(dio3Mask)));
}

Result
Sx126x::clearIrqStatus(const IrqFlags irq)
{
  return toResult(driver::sx126x_clear_irq_status(&m_hal, static_cast<driver::sx126x_irq_mask_t>(irq)));
}

Result
Sx126x::setRfFrequency(const uint32_t frequencyInHz)
{
  return toResult(driver::sx126x_set_rf_freq(&m_hal, frequencyInHz));
}

Result
Sx126x::calibrateImage(const float frequencyMin, const float frequencyMax)
{
  return toResult(
    driver::sx126x_cal_img(&m_hal, static_cast<uint8_t>(frequencyMin), static_cast<uint8_t>(frequencyMax)));
}

Result
Sx126x::setTxParams(const OutputPower outputPower, const PaRampTime rampTime)
{
  const auto result =
    driver::sx126x_set_tx_params(&m_hal, +outputPower, static_cast<driver::sx126x_ramp_time_t>(rampTime));
  if (result == driver::sx126x_status_t::SX126X_STATUS_OK) {
    m_outputPower = outputPower;
  }

  return toResult(result);
}

Result
Sx126x::setModulationParams(const SpreadingFactor spreadingFactor,
                            const Bandwidth bandwidth,
                            const CodingRate codingRate,
                            const bool ldrOptimize)
{
  // calculate symbol length and enable low data rate optimization, if auto-configuration is enabled
  if (m_ldroAuto) {
    const float symbolLength = static_cast<float>(1U << +m_spreadingFactor) / static_cast<float>(toKhz(m_bandwidth));
    m_ldrOptimize = symbolLength >= 16.0F;
  } else {
    m_ldrOptimize = ldrOptimize;
  }

  const driver::sx126x_mod_params_lora_t params{
    .sf = static_cast<driver::sx126x_lora_sf_t>(spreadingFactor),
    .bw = static_cast<driver::sx126x_lora_bw_t>(bandwidth),
    .cr = static_cast<driver::sx126x_lora_cr_t>(codingRate),
    .ldro = static_cast<uint8_t>(m_ldrOptimize ? 1U : 0U),
  };

  return toResult(driver::sx126x_set_lora_mod_params(&m_hal, &params));
}

Result
Sx126x::setPacketParams(const uint16_t preambleLen,
                        const bool crcOn,
                        const uint8_t payloadLen,
                        const PacketLengthMode hdrType,
                        const bool invertIq)
{
  const driver::sx126x_pkt_params_lora_t params{
    .preamble_len_in_symb = preambleLen,
    .header_type = static_cast<driver::sx126x_lora_pkt_len_modes_t>(hdrType),
    .pld_len_in_bytes = payloadLen,
    .crc_is_on = crcOn,
    .invert_iq_is_on = invertIq,
  };

  return toResult(driver::sx126x_set_lora_pkt_params(&m_hal, &params));
}

Result
Sx126x::setBufferBaseAddress(const uint8_t txBaseAddress, const uint8_t rxBaseAddress)
{
  return toResult(driver::sx126x_set_buffer_base_address(&m_hal, txBaseAddress, rxBaseAddress));
}

Result
Sx126x::getPacketStatus(PacketStatus* const status)
{
  driver::sx126x_pkt_status_lora_t pktStatus{};

  const auto result = driver::sx126x_get_lora_pkt_status(&m_hal, &pktStatus);
  if (result == driver::sx126x_status_t::SX126X_STATUS_OK) {
    status->rssiPktInDbm = pktStatus.rssi_pkt_in_dbm;
    status->snrPktInDb = pktStatus.snr_pkt_in_db;
    status->signalRssiPktInDbm = pktStatus.signal_rssi_pkt_in_dbm;
  }
  return toResult(result);
}

DeviceErrors
Sx126x::getDeviceErrors()
{
  driver::sx126x_errors_mask_t errors{};
  driver::sx126x_get_device_errors(&m_hal, &errors);
  return static_cast<DeviceErrors>(errors);
}

Result
Sx126x::clearDeviceErrors()
{
  return toResult(driver::sx126x_clear_device_errors(&m_hal));
}

Result
Sx126x::setFrequencyRaw(const float frequencyInMhz)
{
  // calculate raw value
  m_frequencyInMhz = frequencyInMhz;
  const uint32_t frequencyInHz = static_cast<uint32_t>(frequencyInMhz) * 1'000'000;
  return setRfFrequency(frequencyInHz);
}

Result
Sx126x::fixPaClamping()
{
  return toResult(driver::sx126x_cfg_tx_clamp(&m_hal));
}

float
Sx126x::frequencyInMhz() const
{
  return m_frequencyInMhz;
}

Result
Sx126x::modSetup(const TcxoVoltage tcxoVoltage, const bool useRegulatorLdo) // NOLINT(*-function-cognitive-complexity)
{
  m_hal.init();

  m_hal.pinMode(m_hal.chipSelectPin(), m_hal.gpioModeOutput());
  m_hal.digitalWrite(m_hal.chipSelectPin(), m_hal.gpioLevelHigh());

  m_hal.pinMode(m_hal.irqPin(), m_hal.gpioModeInput());
  m_hal.pinMode(m_hal.busyPin(), m_hal.gpioModeInput());
  // todo implement: parseStatusCb = SPIparseStatus;

  if (not verifyChip()) {
    m_hal.term();
    return Result::ChipNotFound;
  }

  ASSERT(reset());
  ASSERT(standby());

  if (tcxoVoltage != TcxoVoltage::_0V) {
    ASSERT(setTcxo(tcxoVoltage));
  }

  ASSERT(config());

  if (useRegulatorLdo) {
    ASSERT(setRegulatorLdo());
  } else {
    ASSERT(setRegulatorDcdc());
  }

  return Result::Ok;
}

Result
Sx126x::config()
{
  // reset buffer base address
  ASSERT(setBufferBaseAddress());

  ASSERT(toResult(driver::sx126x_set_pkt_type(&m_hal, driver::sx126x_pkt_types_e::SX126X_PKT_TYPE_LORA)));
  ASSERT(toResult(
    driver::sx126x_set_rx_tx_fallback_mode(&m_hal, driver::sx126x_fallback_modes_t::SX126X_FALLBACK_STDBY_RC)));

  const driver::sx126x_cad_params_t params{
    .cad_symb_nb = driver::sx126x_cad_symbs_t::SX126X_CAD_08_SYMB,
    .cad_detect_peak = static_cast<uint8_t>(+m_spreadingFactor + 13U),
    .cad_detect_min = g_cadParamDetMin,
    .cad_exit_mode = driver::sx126x_cad_exit_modes_t::SX126X_CAD_ONLY,
    .cad_timeout = static_cast<uint8_t>(0x00),
  };

  ASSERT(toResult(driver::sx126x_set_cad_params(&m_hal, &params)));

  // clear IRQ
  ASSERT(clearIrqStatus());
  ASSERT(setDioIrqParams(IrqFlags::None, IrqFlags::None));

  // calibrate all blocks
  ASSERT(toResult(driver::sx126x_cal(&m_hal, driver::sx126x_cal_mask_e::SX126X_CAL_ALL)));

  // wait for calibration completion
  m_hal.sleepMs(5);

  while (not m_hal.waitForRadio()) {
  }

  return Result::Ok;
}

bool
Sx126x::verifyChip()
{
  for (size_t index{0U}; index < 10U; ++index) {
    reset();

    char version[16]{};
    driver::sx126x_read_register(
      &m_hal, g_versionRegisterAddress, reinterpret_cast<uint8_t*>(version), sizeof(version));

    if (strncmp(m_chipType, version, sizeof(m_chipType)) == 0) {
      return true;
    }
    m_hal.sleepMs(10U);
  }

  return false;
}

Result
Sx126x::startReceiveCommon(const uint32_t timeout, const IrqFlags irqFlags, IrqFlags irqMask)
{
  // ensure we are in standby
  ASSERT(standby());

  // set DIO mapping
  if (timeout != g_rxTimeoutInf) {
    irqMask = irqMask | IrqFlags::Timeout;
  }
  ASSERT(setDioIrqParams(irqFlags, irqMask)); // NOLINT(*-suspicious-call-argument)

  // set buffer pointers
  ASSERT(setBufferBaseAddress());

  // clear interrupt flags
  ASSERT(clearIrqStatus());

  // restore original packet length
  ASSERT(setPacketParams(m_preambleLength, m_crcEnabled, m_implicitLen, m_headerType, m_invertIqEnabled));

  return Result::Ok;
}

Result
Sx126x::setHeaderType(const PacketLengthMode headerType, const size_t len)
{
  // set requested packet mode
  ASSERT(setPacketParams(m_preambleLength, m_crcEnabled, len, headerType, m_invertIqEnabled));

  // update cached value
  m_headerType = headerType;
  m_implicitLen = len;

  return Result::Ok;
}

Result
Sx126x::fixImplicitTimeout()
{
  // fixes timeout in implicit header mode
  // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.3 for details

  if (m_headerType != PacketLengthMode::Implicit) {
    return Result::WrongModem;
  }

  return toResult(driver::sx126x_stop_rtc(&m_hal));
}
} // namespace sx126x
