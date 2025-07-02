#pragma once

#include <stddef.h>
#include <stdint.h>

#include <etl/array.h>

#include <Arduino.h>

#include <AESLib.h>

namespace aes {
class Aes
{
public:
  using Array = etl::array<byte, N_BLOCK>;

  explicit Aes(const Array& key, paddingMode paddingMode = paddingMode::CMS) noexcept;
  uint16_t encrypt(const byte* input, uint16_t length, byte* output);
  uint16_t encrypt(const char* input, uint16_t length, char* output);
  uint16_t decrypt(byte* input, uint16_t length, byte* output);
  uint16_t decrypt(char* input, uint16_t length, char* output);
  size_t calculateEncryptedLength(int16_t length);

private:
  AESLib m_aesLib;
  Array m_key;
};
} // namespace aes
