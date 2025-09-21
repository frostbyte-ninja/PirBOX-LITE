#include <Arduino.h>

#include <stdio.h>

#include <etl/algorithm.h>

#include <arduino_hal/ArduinoHal.hpp>
#include <crypto/Aes.hpp>
#include <sx126x/Assert.hpp>
#include <sx126x/Sx1262.hpp>
#include <sx126x/Types.hpp>

namespace {
constexpr auto g_gatewayId{"xy"};
constexpr auto g_nodeName{"PirBoxL"};
constexpr float g_loraFrequency{868.0F};
constexpr crypto::Aes::Array
  g_aesKey{0xC5, 0xBD, 0x18, 0x6E, 0x98, 0xBE, 0x79, 0xF3, 0xFA, 0x98, 0xE3, 0x30, 0xF7, 0x1E, 0x4E, 0x93};

constexpr uint8_t g_pirSensorPin{PIN_PC0};
constexpr uint8_t g_powerOffPin{PIN_PB0};
constexpr uint8_t g_batteryPin{PIN_PB4};
constexpr uint8_t g_radioNssPin{PIN_PA4};
constexpr uint8_t g_radioDio1Pin{PIN_PC3};
constexpr uint8_t g_radioResetPin{PIN_PA6};
constexpr uint8_t g_radioBusyPin{PIN_PC2};

// NOLINTBEGIN(*-avoid-non-const-global-variables)

crypto::Aes g_aes{g_aesKey};
sx126x::ArduinoHal g_hal{g_radioNssPin, g_radioDio1Pin, g_radioResetPin, g_radioBusyPin};
sx126x::Sx1262 g_lora{g_hal};
volatile bool g_pirSensorStateChanged{true}; // send the initial state

// NOLINTEND(*-avoid-non-const-global-variables)

void
pirStateChanged()
{
  g_pirSensorStateChanged = true;
}

void
initLoRa()
{
  ASSERT_BLOCKING(g_lora.begin(g_loraFrequency,
                               sx126x::Bandwidth::_125_0,
                               sx126x::SpreadingFactor::_8,
                               sx126x::CodingRate::_4_5,
                               sx126x::g_syncWordPrivate,
                               sx126x::OutputPower::_20Dbm,
                               6U,
                               sx126x::TcxoVoltage::_0V));
  ASSERT_BLOCKING(g_lora.setCrc(true));
  ASSERT_BLOCKING(g_lora.invertIq(false));
  ASSERT_BLOCKING(g_lora.explicitHeader());
}

void
initRandom()
{
  uint32_t value{0U};
  ASSERT_BLOCKING(g_lora.randomInt(value));
  randomSeed(value);
  srand(value);
}

int
readBatteryPercentage()
{
  constexpr size_t samples = 5U;
  constexpr float adcRefVolt = 2.5F;
  constexpr uint8_t adcBits = 12U;
  constexpr int adcResolution = 1U << adcBits;
  constexpr float voltageDivider = 2.0F;
  constexpr float minVolt = 3.3F;
  constexpr float maxVolt = 4.1F;

  // ReSharper disable once CppRedundantParentheses
  constexpr float countsToVolt = (adcRefVolt / static_cast<float>(adcResolution)) * voltageDivider;
  constexpr float voltRange = maxVolt - minVolt;
  constexpr float voltToPct = 100.0F / voltRange;

  int32_t sumRaw = 0;
  for (size_t i = 0; i < samples; ++i) {
    sumRaw += analogReadEnh(g_batteryPin, adcBits);
    delay(2);
  }

  // ReSharper disable once CppRedundantParentheses
  const float avgVolt = (static_cast<float>(sumRaw) * countsToVolt) / samples;
  float pct = (avgVolt - minVolt) * voltToPct;
  pct = etl::clamp(pct, 0.0F, 100.0F);

  return static_cast<int>(pct);
}

void
powerControl(const bool powerOn)
{
  digitalWrite(g_powerOffPin, powerOn ? LOW : HIGH);
}

void
sendMessage(const char* message, const int16_t messageSize)
{
  byte sendBuffer[g_aes.calculateEncryptedLength(messageSize)];
  g_aes.encrypt(reinterpret_cast<const byte*>(message), messageSize, sendBuffer);
  g_lora.transmit(sendBuffer, sizeof(sendBuffer));
}

void
processPirStateChange()
{
  const int8_t motionState = digitalRead(g_pirSensorPin);
  const int battery = readBatteryPercentage();

  char message[128];
  if (const auto messageSize = snprintf(message,
                                        sizeof(message),
                                        R"({"k":"%s","id":"%s","m":"%s","b":%d})",
                                        g_gatewayId,
                                        g_nodeName,
                                        motionState == HIGH ? "on" : "off",
                                        battery);
      messageSize >= 0) {
    sendMessage(message, static_cast<int16_t>(messageSize));
  }
}
} // namespace

void
setup()
{
  pinMode(g_powerOffPin, OUTPUT);
  powerControl(true);

  pinMode(g_pirSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(g_pirSensorPin), pirStateChanged, CHANGE);

  analogReference(INTERNAL2V5); // NOLINT(*-signed-bitwise)

  initLoRa();

  initRandom();
}

void
loop()
{
  if (g_pirSensorStateChanged) {
    g_pirSensorStateChanged = false;
    processPirStateChange();
    powerControl(false);
  }
}
