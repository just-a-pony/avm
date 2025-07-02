/*
 * Copyright (c) 2025, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 3-Clause Clear License
 * and the Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear
 * License was not distributed with this source code in the LICENSE file, you
 * can obtain it at aomedia.org/license/software-license/bsd-3-c-c/.  If the
 * Alliance for Open Media Patent License 1.0 was not distributed with this
 * source code in the PATENTS file, you can obtain it at
 * aomedia.org/license/patent-license/.
 */

#include "third_party/googletest/src/googletest/include/gtest/gtest.h"

#include "aom_dsp/bitwriter_buffer.h"

namespace {

// Test the examples in Table 25 in ITU-T H.274 (V3) (09/2023) and a few more.
//
// Bit string    codeNum
//     1            0
//    010           1
//    011           2
//   00100          3
//   00101          4
//   00110          5
//   00111          6
//  0001000         7
//  0001001         8
//  0001010         9
//  0001011        10
//  0001100        11
//  0001101        12
//  0001110        13
//  0001111        14
TEST(BitwriterBufferTest, UvlcOneByte) {
  static constexpr struct {
    uint32_t bit_offset;
    uint8_t byte;
  } kExpected[] = {
    { 1, 0x80 },  // 0
    { 3, 0x40 },  // 1
    { 3, 0x60 },  // 2
    { 5, 0x20 },  // 3
    { 5, 0x28 },  // 4
    { 5, 0x30 },  // 5
    { 5, 0x38 },  // 6
    { 7, 0x10 },  // 7
    { 7, 0x12 },  // 8
    { 7, 0x14 },  // 9
    { 7, 0x16 },  // 10
    { 7, 0x18 },  // 11
    { 7, 0x1a },  // 12
    { 7, 0x1c },  // 13
    { 7, 0x1e },  // 14
  };
  uint8_t dst[1];

  for (int i = 0; i < 15; i++) {
    struct aom_write_bit_buffer wb = { dst, 0 };
    aom_wb_write_uvlc(&wb, i);
    ASSERT_EQ(wb.bit_offset, kExpected[i].bit_offset);
    EXPECT_EQ(wb.bit_buffer[0], kExpected[i].byte);
  }
}

// Tests two values with the maximum number (31) of leading zero bits.
TEST(BitwriterBufferTest, Uvlc31LeadingZeros) {
  uint8_t dst[8];

  // 2^31 - 1
  {
    struct aom_write_bit_buffer wb = { dst, 0 };
    aom_wb_write_uvlc(&wb, 0x7fffffff);
    ASSERT_EQ(wb.bit_offset, 63u);
    EXPECT_EQ(wb.bit_buffer[0], 0x00);
    EXPECT_EQ(wb.bit_buffer[1], 0x00);
    EXPECT_EQ(wb.bit_buffer[2], 0x00);
    EXPECT_EQ(wb.bit_buffer[3], 0x01);
    EXPECT_EQ(wb.bit_buffer[4], 0x00);
    EXPECT_EQ(wb.bit_buffer[5], 0x00);
    EXPECT_EQ(wb.bit_buffer[6], 0x00);
    EXPECT_EQ(wb.bit_buffer[7], 0x00);
  }

  // 2^32 - 2
  {
    struct aom_write_bit_buffer wb = { dst, 0 };
    aom_wb_write_uvlc(&wb, 0xfffffffe);
    ASSERT_EQ(wb.bit_offset, 63u);
    EXPECT_EQ(wb.bit_buffer[0], 0x00);
    EXPECT_EQ(wb.bit_buffer[1], 0x00);
    EXPECT_EQ(wb.bit_buffer[2], 0x00);
    EXPECT_EQ(wb.bit_buffer[3], 0x01);
    EXPECT_EQ(wb.bit_buffer[4], 0xff);
    EXPECT_EQ(wb.bit_buffer[5], 0xff);
    EXPECT_EQ(wb.bit_buffer[6], 0xff);
    EXPECT_EQ(wb.bit_buffer[7], 0xfe);
  }
}

// Test the examples in Table 25 and Table 26 in ITU-T H.274 (V3) (09/2023) and
// a few more.
//
// Bit string    codeNum    syntax element value
//     1            0                 0
//    010           1                 1
//    011           2                -1
//   00100          3                 2
//   00101          4                -2
//   00110          5                 3
//   00111          6                -3
//  0001000         7                 4
//  0001001         8                -4
//  0001010         9                 5
//  0001011        10                -5
//  0001100        11                 6
//  0001101        12                -6
//  0001110        13                 7
//  0001111        14                -7
TEST(BitwriterBufferTest, SvlcOneByte) {
  static constexpr struct {
    uint32_t bit_offset;
    uint8_t byte;
  } kExpected[] = {
    { 1, 0x80 },  // 0
    { 3, 0x40 },  // 1
    { 3, 0x60 },  // -1
    { 5, 0x20 },  // 2
    { 5, 0x28 },  // -2
    { 5, 0x30 },  // 3
    { 5, 0x38 },  // -3
    { 7, 0x10 },  // 4
    { 7, 0x12 },  // -4
    { 7, 0x14 },  // 5
    { 7, 0x16 },  // -5
    { 7, 0x18 },  // 6
    { 7, 0x1a },  // -6
    { 7, 0x1c },  // 7
    { 7, 0x1e },  // -7
  };
  uint8_t dst[1];

  int32_t sign = 1;
  for (int i = 0; i < 15; i++) {
    // input = (-1)^(i + 1) * Ceil( i / 2.0 )
    sign *= -1;
    const int32_t input = sign * ((i + 1) / 2);
    struct aom_write_bit_buffer wb = { dst, 0 };
    aom_wb_write_svlc(&wb, input);
    ASSERT_EQ(wb.bit_offset, kExpected[i].bit_offset);
    EXPECT_EQ(wb.bit_buffer[0], kExpected[i].byte);
  }
}

// Tests four values with the maximum number (31) of leading zero bits.
TEST(BitwriterBufferTest, Svlc31LeadingZeros) {
  uint8_t dst[8];

  // 2^30
  {
    struct aom_write_bit_buffer wb = { dst, 0 };
    aom_wb_write_svlc(&wb, 1 << 30);
    ASSERT_EQ(wb.bit_offset, 63u);
    EXPECT_EQ(wb.bit_buffer[0], 0x00);
    EXPECT_EQ(wb.bit_buffer[1], 0x00);
    EXPECT_EQ(wb.bit_buffer[2], 0x00);
    EXPECT_EQ(wb.bit_buffer[3], 0x01);
    EXPECT_EQ(wb.bit_buffer[4], 0x00);
    EXPECT_EQ(wb.bit_buffer[5], 0x00);
    EXPECT_EQ(wb.bit_buffer[6], 0x00);
    EXPECT_EQ(wb.bit_buffer[7], 0x00);
  }

  // -2^30
  {
    struct aom_write_bit_buffer wb = { dst, 0 };
    aom_wb_write_svlc(&wb, -(1 << 30));
    ASSERT_EQ(wb.bit_offset, 63u);
    EXPECT_EQ(wb.bit_buffer[0], 0x00);
    EXPECT_EQ(wb.bit_buffer[1], 0x00);
    EXPECT_EQ(wb.bit_buffer[2], 0x00);
    EXPECT_EQ(wb.bit_buffer[3], 0x01);
    EXPECT_EQ(wb.bit_buffer[4], 0x00);
    EXPECT_EQ(wb.bit_buffer[5], 0x00);
    EXPECT_EQ(wb.bit_buffer[6], 0x00);
    EXPECT_EQ(wb.bit_buffer[7], 0x02);
  }

  // 2^31 - 1
  {
    struct aom_write_bit_buffer wb = { dst, 0 };
    aom_wb_write_svlc(&wb, INT32_MAX);
    ASSERT_EQ(wb.bit_offset, 63u);
    EXPECT_EQ(wb.bit_buffer[0], 0x00);
    EXPECT_EQ(wb.bit_buffer[1], 0x00);
    EXPECT_EQ(wb.bit_buffer[2], 0x00);
    EXPECT_EQ(wb.bit_buffer[3], 0x01);
    EXPECT_EQ(wb.bit_buffer[4], 0xff);
    EXPECT_EQ(wb.bit_buffer[5], 0xff);
    EXPECT_EQ(wb.bit_buffer[6], 0xff);
    EXPECT_EQ(wb.bit_buffer[7], 0xfc);
  }

  // -2^31 + 1
  {
    struct aom_write_bit_buffer wb = { dst, 0 };
    aom_wb_write_svlc(&wb, INT32_MIN + 1);
    ASSERT_EQ(wb.bit_offset, 63u);
    EXPECT_EQ(wb.bit_buffer[0], 0x00);
    EXPECT_EQ(wb.bit_buffer[1], 0x00);
    EXPECT_EQ(wb.bit_buffer[2], 0x00);
    EXPECT_EQ(wb.bit_buffer[3], 0x01);
    EXPECT_EQ(wb.bit_buffer[4], 0xff);
    EXPECT_EQ(wb.bit_buffer[5], 0xff);
    EXPECT_EQ(wb.bit_buffer[6], 0xff);
    EXPECT_EQ(wb.bit_buffer[7], 0xfe);
  }
}

}  // namespace
