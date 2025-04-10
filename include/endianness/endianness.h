#pragma once

#include <cstdint>
#include <cstring>

namespace endianness {

/**
   * @brief copies bytes from src to dest while swapping their order
   * 
   * @param dest pointer to the first byte of the destination
   * @param src pointer to the first byte of the source
   * @param size number of bytes to be swapped and copied
   */
inline void swapBytes(const void* src, void* dest, size_t size) {
  const uint8_t* src_bytes = static_cast<const uint8_t*>(src);
  uint8_t* dest_bytes = static_cast<uint8_t*>(dest);
  for (size_t i = 0; i < size; ++i) {
    dest_bytes[i] = src_bytes[size - i - 1];
  }
}

/**
   * @brief copies the bytes from a little-endian source
   * 
   * @param dest pointer to the first byte of the destination
   * @param src pointer the first byte in the source
   * @param size number of bytes to be copied
   */
inline void copyFromLittleEndian(void* dest, const void* src, size_t size) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  std::memcpy(dest, src, size);
#else
  swapBytes(src, dest, size);
#endif
}

/**
   * @brief copies the bytes from a big-endian source
   * 
   * @param src pointer the first byte in the source
   * @param dest pointer to the first byte of the destination
   * @param size number of bytes to be copied
   */
inline void copyFromBigEndian(void* dest, const void* src, size_t size) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::memcpy(dest, src, size);
#else
  swapBytes(src, dest, size);
#endif
}

/**
   * @brief stores the bytes to a big-endian source
   * 
   * @param src pointer the first byte in the source
   * @param dest pointer to the first byte of the destination
   * @param size number of bytes to be copied
   */
inline void storeAsBigEndian(void* dest, const void* src, size_t size) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::memcpy(dest, src, size);
#else
  swapBytes(src, dest, size);
#endif
}

/**
   * @brief stores the bytes to a little-endian source
   * 
   * @param src pointer the first byte in the source
   * @param dest pointer to the first byte of the destination
   * @param size number of bytes to be copied
   */
inline void storeAsLittleEndian(void* dest, const void* src, size_t size) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  std::memcpy(dest, src, size);
#else
  swapBytes(src, dest, size);
#endif
}

}  // namespace endianness
