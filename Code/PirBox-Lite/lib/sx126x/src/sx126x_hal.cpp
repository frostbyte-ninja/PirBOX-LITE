#include <sx126x_hal.h>

#include <sx126x/IHal.hpp>

using namespace sx126x;

namespace {
const IHal&
toHal(const void* context)
{
  return *static_cast<const IHal*>(context);
}

constexpr uint8_t g_sleepCommand{0x84U};
} // namespace

sx126x_hal_status_t
sx126x_hal_write(const void* context,
                 const uint8_t* command,
                 const uint16_t command_length, // NOLINT(*-identifier-naming)
                 const uint8_t* data,
                 const uint16_t data_length) // NOLINT(*-identifier-naming)
{
  const bool waitForRadio = command[0] != g_sleepCommand;
  return toHal(context).write(command, command_length, data, data_length, waitForRadio);
}

sx126x_hal_status_t
sx126x_hal_read(const void* context,
                const uint8_t* command,
                const uint16_t command_length, // NOLINT(*-identifier-naming)
                uint8_t* data,
                const uint16_t data_length) // NOLINT(*-identifier-naming)
{
  return toHal(context).read(command, command_length, data, data_length, true);
}

sx126x_hal_status_t
sx126x_hal_reset(const void* context)
{
  return toHal(context).reset();
}

sx126x_hal_status_t
sx126x_hal_wakeup(const void* context)
{
  return toHal(context).wakeup();
}
