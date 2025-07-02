#include <aes/Aes.hpp>

namespace aes {
Aes::Aes(const Array& key, paddingMode paddingMode) noexcept
  : m_key{key}
{
  m_aesLib.set_paddingmode(paddingMode);
}

uint16_t
Aes::encrypt(const byte* input, const uint16_t length, byte* const output)
{
  Array aesIv;
  m_aesLib.gen_iv(aesIv.data());
  memcpy(output, aesIv.data(), aesIv.size());

  return m_aesLib.encrypt(input, length, output + aesIv.size(), m_key.data(), sizeof(m_key), aesIv.data()) +
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
    input + aesIv.size(), length - aesIv.size(), output, m_key.data(), sizeof(m_key), aesIv.data());
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
} // namespace aes
