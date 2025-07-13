#pragma once

#include <stdint.h>

/// @brief 32bit変数のバイトオーダーを変換
/// @tparam T 変換する変数の型
/// @param val 変数のポインタ
template<typename T>
void swap32(T *val) {
  uint8_t *u8 = (uint8_t *)val;
  uint8_t tmp;
  // swap the bytes into a temporary buffer
  tmp = u8[0];
  u8[0] = u8[3];
  u8[3] = tmp;
  tmp = u8[1];
  u8[1] = u8[2];
  u8[2] = tmp;
}

/// @brief 64bit変数のバイトオーダーを変換
/// @tparam T 変換する変数の型
/// @param val 変数のポインタ
template<typename T>
void swap64(T *val) {
  uint8_t *u8 = (uint8_t *)val;
  uint8_t tmp;
  // swap the bytes into a temporary buffer
  tmp = u8[0];
  u8[0] = u8[7];
  u8[7] = tmp;

  tmp = u8[1];
  u8[1] = u8[6];
  u8[6] = tmp;

  tmp = u8[2];
  u8[2] = u8[5];
  u8[5] = tmp;

  tmp = u8[3];
  u8[3] = u8[4];
  u8[4] = tmp;
}
