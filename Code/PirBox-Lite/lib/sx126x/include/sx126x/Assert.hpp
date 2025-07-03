#pragma once

// NOLINTBEGIN(*-macro-usage)

#define ASSERT(EXPR)                                                                                                   \
  do {                                                                                                                 \
    if (const sx126x::Result _res = (EXPR); _res != sx126x::Result::Ok) {                                              \
      return _res;                                                                                                     \
    }                                                                                                                  \
  } while (false)

#define ASSERT_BLOCKING(EXPR)                                                                                          \
  do {                                                                                                                 \
    if (const sx126x::Result _res = (EXPR); _res != sx126x::Result::Ok) {                                              \
      while (true) {                                                                                                   \
      }                                                                                                                \
    }                                                                                                                  \
  } while (false)

#define ASSERT_PTR(PTR)                                                                                                \
  do {                                                                                                                 \
    if ((PTR) == nullptr) {                                                                                            \
      return Result::MemoryAllocationFailed;                                                                           \
    }                                                                                                                  \
  } while (false)

#define ASSERT_RANGE(VAR, MIN, MAX, ERR)                                                                               \
  do {                                                                                                                 \
    if ((VAR) < (MIN) or (VAR) > (MAX)) {                                                                              \
      return ERR;                                                                                                      \
    }                                                                                                                  \
  } while (false)

// NOLINTEND(*-macro-usage)
