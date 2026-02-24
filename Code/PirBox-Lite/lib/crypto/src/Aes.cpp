#include <crypto/Aes.hpp>

#include <etl/algorithm.h>
#include <etl/limits.h>

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

namespace crypto {
namespace {
void
fillIv(Aes::Array& aesIv)
{
  using ValueType = Aes::Array::value_type;
  etl::generate(aesIv.begin(), aesIv.end(), [] {
    return static_cast<ValueType>(random(etl::numeric_limits<ValueType>::min(), etl::numeric_limits<ValueType>::max()));
  });
}
} // namespace

Aes::Aes(const Array& key, const paddingMode paddingMode) noexcept
  : m_key{key}
{
  m_aesLib.set_paddingmode(paddingMode);
}

uint16_t
Aes::encrypt(const byte* input, const uint16_t length, byte* const output)
{
  Array aesIv;
  fillIv(aesIv);
  memcpy(output, aesIv.data(), aesIv.size());

  return m_aesLib.encrypt(
           input, length, output + aesIv.size(), m_key.data(), static_cast<int>(m_key.size()), aesIv.data()) +
         aesIv.size();
}

uint16_t
Aes::encrypt(const char* input, const uint16_t length, char* const output)
{
  return encrypt(reinterpret_cast<const byte*>(input), length, reinterpret_cast<byte*>(output));
}

uint16_t
Aes::decrypt(byte* const input, const uint16_t length, byte* const output)
{
  Array aesIv;
  if (length < aesIv.size()) {
    return 0; // Not enough data for IV
  }
  memcpy(aesIv.data(), input, aesIv.size());

  return m_aesLib.decrypt(
    input + aesIv.size(), length - aesIv.size(), output, m_key.data(), static_cast<int>(m_key.size()), aesIv.data());
}

uint16_t
Aes::decrypt(char* const input, const uint16_t length, char* const output)
{
  return decrypt(reinterpret_cast<byte*>(input), length, reinterpret_cast<byte*>(output));
}

size_t
Aes::calculateEncryptedLength(const int16_t length)
{
  return N_BLOCK + m_aesLib.get_cipher_length(length);
}
} // namespace crypto
