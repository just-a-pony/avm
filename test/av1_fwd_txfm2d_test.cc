/*
 * Copyright (c) 2021, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 3-Clause Clear License
 * and the Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear
 * License was not distributed with this source code in the LICENSE file, you
 * can obtain it at aomedia.org/license/software-license/bsd-3-c-c/.  If the
 * Alliance for Open Media Patent License 1.0 was not distributed with this
 * source code in the PATENTS file, you can obtain it at
 * aomedia.org/license/patent-license/.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <tuple>
#include <vector>

#include "config/av1_rtcd.h"

#include "test/acm_random.h"
#include "test/util.h"
#include "test/av1_txfm_test.h"
#include "test/function_equivalence_test.h"
#include "av1/common/av1_txfm.h"
#include "av1/encoder/hybrid_fwd_txfm.h"

using libaom_test::ACMRandom;
using libaom_test::bd;
using libaom_test::compute_avg_abs_error;
using libaom_test::input_base;
using libaom_test::TYPE_TXFM;

using std::vector;

namespace {
// tx_type_, tx_size_, max_error_, max_avg_error_
typedef std::tuple<TX_TYPE, TX_SIZE, double, double> AV1FwdTxfm2dParam;

class AV1FwdTxfm2d : public ::testing::TestWithParam<AV1FwdTxfm2dParam> {
 public:
  virtual void SetUp() {
    tx_type_ = GET_PARAM(0);
    tx_size_ = GET_PARAM(1);
    max_error_ = GET_PARAM(2);
    max_avg_error_ = GET_PARAM(3);
#if CONFIG_INTER_DDT
    count_ = 1000;
#else
    count_ = 500;
#endif  // CONFIG_INTER_DDT
    TXFM_2D_FLIP_CFG fwd_txfm_flip_cfg;
    av1_get_fwd_txfm_cfg(tx_type_, tx_size_, &fwd_txfm_flip_cfg);
    amplify_factor_ = libaom_test::get_amplification_factor(tx_type_, tx_size_);
    tx_width_ = tx_size_wide[fwd_txfm_flip_cfg.tx_size];
    tx_height_ = tx_size_high[fwd_txfm_flip_cfg.tx_size];
    ud_flip_ = fwd_txfm_flip_cfg.ud_flip;
    lr_flip_ = fwd_txfm_flip_cfg.lr_flip;

    fwd_txfm_ = libaom_test::fwd_txfm_func_ls[tx_size_];
    txfm2d_size_ = tx_width_ * tx_height_;
    input_ = reinterpret_cast<int16_t *>(
        aom_memalign(16, sizeof(input_[0]) * txfm2d_size_));
    output_ = reinterpret_cast<int32_t *>(
        aom_memalign(16, sizeof(output_[0]) * txfm2d_size_));
    ref_input_ = reinterpret_cast<double *>(
        aom_memalign(16, sizeof(ref_input_[0]) * txfm2d_size_));
    ref_output_ = reinterpret_cast<double *>(
        aom_memalign(16, sizeof(ref_output_[0]) * txfm2d_size_));
  }

  void RunFwdAccuracyCheck() {
    ACMRandom rnd(ACMRandom::DeterministicSeed());
    double avg_abs_error = 0;
    for (int ci = 0; ci < count_; ci++) {
      for (int ni = 0; ni < txfm2d_size_; ++ni) {
        input_[ni] = rnd.Rand16() % input_base;
        ref_input_[ni] = static_cast<double>(input_[ni]);
        output_[ni] = 0;
        ref_output_[ni] = 0;
      }

      fwd_txfm_(input_, output_, tx_width_, tx_type_,
#if CONFIG_INTER_DDT
                ci % 2,
#endif  // CONFIG_INTER_DDT
                bd);

      if (lr_flip_ && ud_flip_) {
        libaom_test::fliplrud(ref_input_, tx_width_, tx_height_, tx_width_);
      } else if (lr_flip_) {
        libaom_test::fliplr(ref_input_, tx_width_, tx_height_, tx_width_);
      } else if (ud_flip_) {
        libaom_test::flipud(ref_input_, tx_width_, tx_height_, tx_width_);
      }

      libaom_test::reference_hybrid_2d(ref_input_, ref_output_, tx_type_,
#if CONFIG_INTER_DDT
                                       ci % 2,
#endif  // CONFIG_INTER_DDT
                                       tx_size_);

      double actual_max_error = 0;
      for (int ni = 0; ni < txfm2d_size_; ++ni) {
        ref_output_[ni] = round(ref_output_[ni]);
        const double this_error =
            fabs(output_[ni] - ref_output_[ni]) / amplify_factor_;
        actual_max_error = AOMMAX(actual_max_error, this_error);
      }
      EXPECT_GE(max_error_, actual_max_error)
          << "tx_size = " << tx_size_ << ", tx_type = " << tx_type_;
      if (actual_max_error > max_error_) {  // exit early.
        break;
      }

      avg_abs_error += compute_avg_abs_error<int32_t, double>(
          output_, ref_output_, txfm2d_size_);
    }

    avg_abs_error /= amplify_factor_;
    avg_abs_error /= count_;
    EXPECT_GE(max_avg_error_, avg_abs_error)
        << "tx_size = " << tx_size_ << ", tx_type = " << tx_type_;
  }

  virtual void TearDown() {
    aom_free(input_);
    aom_free(output_);
    aom_free(ref_input_);
    aom_free(ref_output_);
  }

 private:
  double max_error_;
  double max_avg_error_;
  int count_;
  double amplify_factor_;
  TX_TYPE tx_type_;
  TX_SIZE tx_size_;
  int tx_width_;
  int tx_height_;
  int txfm2d_size_;
  FwdTxfm2dFunc fwd_txfm_;
  int16_t *input_;
  int32_t *output_;
  double *ref_input_;
  double *ref_output_;
  int ud_flip_;  // flip upside down
  int lr_flip_;  // flip left to right
};

static double avg_error_ls[TX_SIZES_ALL] = {
  0.66,  // 4x4 transform
  0.5,   // 8x8 transform
  1.2,   // 16x16 transform
  6.1,   // 32x32 transform
  3.4,   // 64x64 transform
  0.57,  // 4x8 transform
  0.68,  // 8x4 transform
  0.92,  // 8x16 transform
  1.1,   // 16x8 transform
  4.1,   // 16x32 transform
  6,     // 32x16 transform
  3.5,   // 32x64 transform
  5.7,   // 64x32 transform
  0.6,   // 4x16 transform
  0.9,   // 16x4 transform
  1.2,   // 8x32 transform
  1.7,   // 32x8 transform
  2.0,   // 16x64 transform
  4.7,   // 64x16 transform
  0.8,   // 4x32 transform
  0.8,   // 32x4 transform
  4.0,   // 8x64 transform
  4.0,   // 64x8 transform
  3.0,   // 4x64 transform
  3.0,   // 64x4 transform
};

static double max_error_ls[TX_SIZES_ALL] = {
  3.5,  // 4x4 transform
  5,    // 8x8 transform
  11,   // 16x16 transform
  70,   // 32x32 transform
  64,   // 64x64 transform
  3.9,  // 4x8 transform
  4.3,  // 8x4 transform
  12,   // 8x16 transform
  12,   // 16x8 transform
  32,   // 16x32 transform
  46,   // 32x16 transform
  136,  // 32x64 transform
  136,  // 64x32 transform
  5,    // 4x16 transform
  6,    // 16x4 transform
  21,   // 8x32 transform
  13,   // 32x8 transform
  30,   // 16x64 transform
  36,   // 64x16 transform
  8,    // 4x32 transform
  8,    // 32x4 transform
  99,   // 8x64 transform
  99,   // 64x8 transform
  90,   // 4x64 transform
  90,   // 64x4 transform
};

vector<AV1FwdTxfm2dParam> GetTxfm2dParamList() {
  vector<AV1FwdTxfm2dParam> param_list;
  for (int s = 0; s < TX_SIZES; ++s) {
    const double max_error = max_error_ls[s];
    const double avg_error = avg_error_ls[s];
    for (int t = 0; t < TX_TYPES; ++t) {
      const TX_TYPE tx_type = static_cast<TX_TYPE>(t);
      const TX_SIZE tx_size = static_cast<TX_SIZE>(s);
      if (libaom_test::IsTxSizeTypeValid(tx_size, tx_type)) {
        param_list.push_back(
            AV1FwdTxfm2dParam(tx_type, tx_size, max_error, avg_error));
      }
    }
  }
  return param_list;
}

INSTANTIATE_TEST_SUITE_P(C, AV1FwdTxfm2d,
                         ::testing::ValuesIn(GetTxfm2dParamList()));

TEST_P(AV1FwdTxfm2d, RunFwdAccuracyCheck) { RunFwdAccuracyCheck(); }

TEST(AV1FwdTxfm2d, CfgTest) {
  for (int bd_idx = 0; bd_idx < BD_NUM; ++bd_idx) {
    int bd = libaom_test::bd_arr[bd_idx];
    int8_t low_range = libaom_test::low_range_arr[bd_idx];
    int8_t high_range = libaom_test::high_range_arr[bd_idx];
    for (int tx_size = 0; tx_size < TX_SIZES_ALL; ++tx_size) {
      for (int tx_type = 0; tx_type < TX_TYPES; ++tx_type) {
        if (libaom_test::IsTxSizeTypeValid(static_cast<TX_SIZE>(tx_size),
                                           static_cast<TX_TYPE>(tx_type)) ==
            false) {
          continue;
        }
        TXFM_2D_FLIP_CFG cfg;
        av1_get_fwd_txfm_cfg(static_cast<TX_TYPE>(tx_type),
                             static_cast<TX_SIZE>(tx_size), &cfg);
        int8_t stage_range_col[MAX_TXFM_STAGE_NUM];
        int8_t stage_range_row[MAX_TXFM_STAGE_NUM];
        av1_gen_fwd_stage_range(stage_range_col, stage_range_row, &cfg, bd);
        libaom_test::txfm_stage_range_check(stage_range_col, cfg.stage_num_col,
                                            cfg.cos_bit_col, low_range,
                                            high_range);
        libaom_test::txfm_stage_range_check(stage_range_row, cfg.stage_num_row,
                                            cfg.cos_bit_row, low_range,
                                            high_range);
      }
    }
  }
}

typedef void (*lowbd_fwd_txfm_func)(const int16_t *src_diff, tran_low_t *coeff,
                                    int diff_stride, TxfmParam *txfm_param);

void AV1FwdTxfm2dMatchTest(TX_SIZE tx_size, lowbd_fwd_txfm_func target_func) {
  const int bd = 8;
  TxfmParam param;
  memset(&param, 0, sizeof(param));
  const int rows = tx_size_high[tx_size];
  const int cols = tx_size_wide[tx_size];
  // printf("%d x %d\n", cols, rows);
  for (int tx_type = 0; tx_type < TX_TYPES; ++tx_type) {
    if (libaom_test::IsTxSizeTypeValid(
            tx_size, static_cast<TX_TYPE>(tx_type)) == false) {
      continue;
    }

    FwdTxfm2dFunc ref_func = libaom_test::fwd_txfm_func_ls[tx_size];
    if (ref_func != NULL) {
      DECLARE_ALIGNED(32, int16_t, input[64 * 64]) = { 0 };
      DECLARE_ALIGNED(32, int32_t, output[64 * 64]);
      DECLARE_ALIGNED(32, int32_t, ref_output[64 * 64]);
      int input_stride = 64;
      ACMRandom rnd(ACMRandom::DeterministicSeed());
#if CONFIG_INTER_DDT
      for (int cnt = 0; cnt < 1000; ++cnt) {
#else
      for (int cnt = 0; cnt < 500; ++cnt) {
#endif  // CONFIG_INTER_DDT
        if (cnt == 0) {
          for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
              input[r * input_stride + c] = (1 << bd) - 1;
            }
          }
        } else {
          for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
              input[r * input_stride + c] = rnd.Rand16() % (1 << bd);
            }
          }
        }
        param.tx_type = (TX_TYPE)tx_type;
        param.tx_size = (TX_SIZE)tx_size;
        param.tx_set_type = EXT_TX_SET_ALL16;
        param.bd = bd;
#if CONFIG_INTER_DDT
        param.use_ddt = cnt % 2;
#endif  // CONFIG_INTER_DDT
        ref_func(input, ref_output, input_stride, (TX_TYPE)tx_type,
#if CONFIG_INTER_DDT
                 cnt % 2,
#endif  // CONFIG_INTER_DDT
                 bd);
        target_func(input, output, input_stride, &param);
        const int check_rows = AOMMIN(32, rows);
        const int check_cols = AOMMIN(32, rows * cols / check_rows);
        for (int r = 0; r < check_rows; ++r) {
          for (int c = 0; c < check_cols; ++c) {
            ASSERT_EQ(ref_output[r * check_cols + c],
                      output[r * check_cols + c])
                << "[" << r << "," << c << "] cnt:" << cnt
                << " tx_size: " << (int)tx_size << " tx_type: " << tx_type;
          }
        }
      }
    }
  }
}

void AV1FwdTxfm2dSpeedTest(TX_SIZE tx_size, lowbd_fwd_txfm_func target_func) {
  TxfmParam param;
  memset(&param, 0, sizeof(param));
  const int rows = tx_size_high[tx_size];
  const int cols = tx_size_wide[tx_size];
#if CONFIG_INTER_DDT
  const int num_loops = 2000000 / (rows * cols);
#else
  const int num_loops = 1000000 / (rows * cols);
#endif  // CONFIG_INTER_DDT

  for (int i = 0; i < 2; ++i) {
    const int bd = 8;
    for (int tx_type = 0; tx_type < TX_TYPES; ++tx_type) {
      if (libaom_test::IsTxSizeTypeValid(
              tx_size, static_cast<TX_TYPE>(tx_type)) == false) {
        continue;
      }

      FwdTxfm2dFunc ref_func = libaom_test::fwd_txfm_func_ls[tx_size];
      if (ref_func != NULL) {
        DECLARE_ALIGNED(32, int16_t, input[64 * 64]) = { 0 };
        DECLARE_ALIGNED(32, int32_t, output[64 * 64]);
        DECLARE_ALIGNED(32, int32_t, ref_output[64 * 64]);
        int input_stride = 64;
        ACMRandom rnd(ACMRandom::DeterministicSeed());

        for (int r = 0; r < rows; ++r) {
          for (int c = 0; c < cols; ++c) {
            input[r * input_stride + c] = rnd.Rand16() % (1 << bd);
          }
        }

        param.tx_type = (TX_TYPE)tx_type;
        param.tx_size = (TX_SIZE)tx_size;
        param.tx_set_type = EXT_TX_SET_ALL16;
        param.bd = bd;

        aom_usec_timer ref_timer, test_timer;

        aom_usec_timer_start(&ref_timer);
        for (int i = 0; i < num_loops; ++i) {
          ref_func(input, ref_output, input_stride, (TX_TYPE)tx_type,
#if CONFIG_INTER_DDT
                   i % 2,
#endif  // CONFIG_INTER_DDT
                   bd);
        }
        aom_usec_timer_mark(&ref_timer);
        const int elapsed_time_c =
            static_cast<int>(aom_usec_timer_elapsed(&ref_timer));

        aom_usec_timer_start(&test_timer);
        for (int i = 0; i < num_loops; ++i) {
#if CONFIG_INTER_DDT
          param.use_ddt = i % 2;
#endif  // CONFIG_INTER_DDT
          target_func(input, output, input_stride, &param);
        }
        aom_usec_timer_mark(&test_timer);
        const int elapsed_time_simd =
            static_cast<int>(aom_usec_timer_elapsed(&test_timer));

        printf(
            "txfm_size[%d] \t txfm_type[%d] \t c_time=%d \t simd_time=%d \t "
            "gain=%d \n",
            tx_size, tx_type, elapsed_time_c, elapsed_time_simd,
            (elapsed_time_c / elapsed_time_simd));
      }
    }
  }
}

typedef std::tuple<TX_SIZE, lowbd_fwd_txfm_func> LbdFwdTxfm2dParam;

class AV1FwdTxfm2dTest : public ::testing::TestWithParam<LbdFwdTxfm2dParam> {};
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AV1FwdTxfm2dTest);

TEST_P(AV1FwdTxfm2dTest, match) {
  AV1FwdTxfm2dMatchTest(GET_PARAM(0), GET_PARAM(1));
}
TEST_P(AV1FwdTxfm2dTest, DISABLED_Speed) {
  AV1FwdTxfm2dSpeedTest(GET_PARAM(0), GET_PARAM(1));
}
using ::testing::Combine;
using ::testing::Values;
using ::testing::ValuesIn;

#if HAVE_SSE2
/* clang-format off */
static TX_SIZE fwd_txfm_for_sse2[] = {
  TX_4X4,  TX_8X8,  TX_16X16, TX_32X32, TX_64X64, TX_4X8,   TX_8X4,
  TX_8X16, TX_16X8, TX_16X32, TX_32X16, TX_32X64, TX_64X32, TX_4X16,
  TX_16X4, TX_8X32, TX_32X8,  TX_16X64, TX_64X16,
  TX_4X32, TX_32X4, TX_8X64,  TX_64X8,  TX_4X64,  TX_64X4,
};
/* clang-format on */

INSTANTIATE_TEST_SUITE_P(SSE2, AV1FwdTxfm2dTest,
                         Combine(ValuesIn(fwd_txfm_for_sse2),
                                 Values(av1_lowbd_fwd_txfm_sse2)));
#endif  // HAVE_SSE2

#if HAVE_SSE4_1
static TX_SIZE fwd_txfm_for_sse41[] = {
  TX_4X4,
  TX_64X64,
  TX_32X64,
  TX_64X32,
};

INSTANTIATE_TEST_SUITE_P(SSE4_1, AV1FwdTxfm2dTest,
                         Combine(ValuesIn(fwd_txfm_for_sse41),
                                 Values(av1_lowbd_fwd_txfm_sse4_1)));
#endif  // HAVE_SSE4_1

#if HAVE_AVX2
/* clang-format off */
static TX_SIZE fwd_txfm_for_avx2[] = {
  TX_4X4,  TX_8X8,  TX_16X16, TX_32X32, TX_64X64, TX_4X8,   TX_8X4,
  TX_8X16, TX_16X8, TX_16X32, TX_32X16, TX_32X64, TX_64X32, TX_4X16,
  TX_16X4, TX_8X32, TX_32X8,  TX_16X64, TX_64X16,
  TX_4X32, TX_32X4, TX_8X64,  TX_64X8,  TX_4X64,  TX_64X4,
};
/* clang-format on */

INSTANTIATE_TEST_SUITE_P(AVX2, AV1FwdTxfm2dTest,
                         Combine(ValuesIn(fwd_txfm_for_avx2),
                                 Values(av1_lowbd_fwd_txfm_avx2)));
#endif  // HAVE_AVX2

#if HAVE_NEON && !CONFIG_ADST_TUNED && !CONFIG_INTER_DDT

static TX_SIZE fwd_txfm_for_neon[] = { TX_4X4,   TX_8X8,   TX_16X16, TX_32X32,
                                       TX_64X64, TX_4X8,   TX_8X4,   TX_8X16,
                                       TX_16X8,  TX_16X32, TX_32X16, TX_32X64,
                                       TX_64X32, TX_4X16,  TX_16X4,  TX_8X32,
                                       TX_32X8,  TX_16X64, TX_64X16 };

INSTANTIATE_TEST_SUITE_P(NEON, AV1FwdTxfm2dTest,
                         Combine(ValuesIn(fwd_txfm_for_neon),
                                 Values(av1_lowbd_fwd_txfm_neon)));

#endif  // HAVE_NEON && !CONFIG_ADST_TUNED && !CONFIG_INTER_DDT

typedef void (*Highbd_fwd_txfm_func)(const int16_t *src_diff, tran_low_t *coeff,
                                     int diff_stride, TxfmParam *txfm_param);

void AV1HighbdFwdTxfm2dMatchTest(TX_SIZE tx_size,
                                 Highbd_fwd_txfm_func target_func) {
  const int bd_ar[2] = { 10, 12 };
  TxfmParam param;
  memset(&param, 0, sizeof(param));
  const int rows = tx_size_high[tx_size];
  const int cols = tx_size_wide[tx_size];
  for (int i = 0; i < 2; ++i) {
    const int bd = bd_ar[i];
    for (int tx_type = 0; tx_type < TX_TYPES; ++tx_type) {
      if (libaom_test::IsTxSizeTypeValid(
              tx_size, static_cast<TX_TYPE>(tx_type)) == false) {
        continue;
      }

      FwdTxfm2dFunc ref_func = libaom_test::fwd_txfm_func_ls[tx_size];
      if (ref_func != NULL) {
        DECLARE_ALIGNED(32, int16_t, input[64 * 64]) = { 0 };
        DECLARE_ALIGNED(32, int32_t, output[64 * 64]);
        DECLARE_ALIGNED(32, int32_t, ref_output[64 * 64]);
        int input_stride = 64;
        ACMRandom rnd(ACMRandom::DeterministicSeed());
#if CONFIG_INTER_DDT
        for (int cnt = 0; cnt < 1000; ++cnt) {
#else
        for (int cnt = 0; cnt < 500; ++cnt) {
#endif  // CONFIG_INTER_DDT
          if (cnt == 0) {
            for (int r = 0; r < rows; ++r) {
              for (int c = 0; c < cols; ++c) {
                input[r * input_stride + c] = (1 << bd) - 1;
              }
            }
          } else {
            for (int r = 0; r < rows; ++r) {
              for (int c = 0; c < cols; ++c) {
                input[r * input_stride + c] = rnd.Rand16() % (1 << bd);
              }
            }
          }
          param.tx_type = (TX_TYPE)tx_type;
          param.tx_size = (TX_SIZE)tx_size;
          param.tx_set_type = EXT_TX_SET_ALL16;
          param.bd = bd;
#if CONFIG_INTER_DDT
          param.use_ddt = cnt % 2;
#endif  // CONFIG_INTER_DDT

          ref_func(input, ref_output, input_stride, (TX_TYPE)tx_type,
#if CONFIG_INTER_DDT
                   cnt % 2,
#endif  // CONFIG_INTER_DDT
                   bd);
          target_func(input, output, input_stride, &param);
          const int check_rows = AOMMIN(32, rows);
          const int check_cols = AOMMIN(32, rows * cols / check_rows);
          for (int r = 0; r < check_rows; ++r) {
            for (int c = 0; c < check_cols; ++c) {
              ASSERT_EQ(ref_output[r * check_cols + c],
                        output[r * check_cols + c])
                  << "[" << r << "," << c << "] cnt:" << cnt
                  << " tx_size: " << (int)tx_size << " tx_type: " << tx_type;
            }
          }
        }
      }
    }
  }
}

void AV1HighbdFwdTxfm2dSpeedTest(TX_SIZE tx_size,
                                 Highbd_fwd_txfm_func target_func) {
  const int bd_ar[2] = { 10, 12 };
  TxfmParam param;
  memset(&param, 0, sizeof(param));
  const int rows = tx_size_high[tx_size];
  const int cols = tx_size_wide[tx_size];
#if CONFIG_INTER_DDT
  const int num_loops = 2000000 / (rows * cols);
#else
  const int num_loops = 1000000 / (rows * cols);
#endif  // CONFIG_INTER_DDT

  for (int i = 0; i < 2; ++i) {
    const int bd = bd_ar[i];
    for (int tx_type = 0; tx_type < TX_TYPES; ++tx_type) {
      if (libaom_test::IsTxSizeTypeValid(
              tx_size, static_cast<TX_TYPE>(tx_type)) == false) {
        continue;
      }

      FwdTxfm2dFunc ref_func = libaom_test::fwd_txfm_func_ls[tx_size];
      if (ref_func != NULL) {
        DECLARE_ALIGNED(32, int16_t, input[64 * 64]) = { 0 };
        DECLARE_ALIGNED(32, int32_t, output[64 * 64]);
        DECLARE_ALIGNED(32, int32_t, ref_output[64 * 64]);
        int input_stride = 64;
        ACMRandom rnd(ACMRandom::DeterministicSeed());

        for (int r = 0; r < rows; ++r) {
          for (int c = 0; c < cols; ++c) {
            input[r * input_stride + c] = rnd.Rand16() % (1 << bd);
          }
        }

        param.tx_type = (TX_TYPE)tx_type;
        param.tx_size = (TX_SIZE)tx_size;
        param.tx_set_type = EXT_TX_SET_ALL16;
        param.bd = bd;

        aom_usec_timer ref_timer, test_timer;

        aom_usec_timer_start(&ref_timer);
        for (int i = 0; i < num_loops; ++i) {
          ref_func(input, ref_output, input_stride, (TX_TYPE)tx_type,
#if CONFIG_INTER_DDT
                   i % 2,
#endif  // CONFIG_INTER_DDT
                   bd);
        }
        aom_usec_timer_mark(&ref_timer);
        const int elapsed_time_c =
            static_cast<int>(aom_usec_timer_elapsed(&ref_timer));

        aom_usec_timer_start(&test_timer);
        for (int i = 0; i < num_loops; ++i) {
#if CONFIG_INTER_DDT
          param.use_ddt = i % 2;
#endif  // CONFIG_INTER_DDT
          target_func(input, output, input_stride, &param);
        }
        aom_usec_timer_mark(&test_timer);
        const int elapsed_time_simd =
            static_cast<int>(aom_usec_timer_elapsed(&test_timer));

        printf(
            "txfm_size[%d] \t txfm_type[%d] \t c_time=%d \t simd_time=%d \t "
            "gain=%.3f \n",
            tx_size, tx_type, elapsed_time_c, elapsed_time_simd,
            ((float)elapsed_time_c / elapsed_time_simd));
      }
    }
  }
}

typedef std::tuple<TX_SIZE, Highbd_fwd_txfm_func> HighbdFwdTxfm2dParam;

class AV1HighbdFwdTxfm2dTest
    : public ::testing::TestWithParam<HighbdFwdTxfm2dParam> {};
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AV1HighbdFwdTxfm2dTest);

TEST_P(AV1HighbdFwdTxfm2dTest, match) {
  AV1HighbdFwdTxfm2dMatchTest(GET_PARAM(0), GET_PARAM(1));
}

TEST_P(AV1HighbdFwdTxfm2dTest, DISABLED_Speed) {
  AV1HighbdFwdTxfm2dSpeedTest(GET_PARAM(0), GET_PARAM(1));
}

using ::testing::Combine;
using ::testing::Values;
using ::testing::ValuesIn;

#if HAVE_SSE4_1
/* clang-format off */
static TX_SIZE Highbd_fwd_txfm_for_sse4_1[] = {
  TX_4X4,  TX_8X8,  TX_16X16, TX_32X32, TX_64X64, TX_4X8,   TX_8X4,
  TX_8X16, TX_16X8, TX_16X32, TX_32X16, TX_32X64, TX_64X32, TX_4X16,
  TX_16X4, TX_8X32, TX_32X8,  TX_16X64, TX_64X16,
  TX_4X32, TX_32X4, TX_8X64,  TX_64X8,  TX_4X64,  TX_64X4,
};
/* clang-format on */

INSTANTIATE_TEST_SUITE_P(SSE4_1, AV1HighbdFwdTxfm2dTest,
                         Combine(ValuesIn(Highbd_fwd_txfm_for_sse4_1),
                                 Values(av1_highbd_fwd_txfm)));
#endif  // HAVE_SSE4_1
#if HAVE_AVX2
/* clang-format off */
static TX_SIZE Highbd_fwd_txfm_for_avx2[] = {
  TX_8X8,   TX_16X16, TX_32X32, TX_64X64, TX_8X16, TX_16X8,
  TX_16X32, TX_32X16, TX_32X64, TX_64X32, TX_8X32, TX_32X8,
  TX_16X64, TX_64X16, TX_8X64,  TX_64X8
};
/* clang-format on */

INSTANTIATE_TEST_SUITE_P(AVX2, AV1HighbdFwdTxfm2dTest,
                         Combine(ValuesIn(Highbd_fwd_txfm_for_avx2),
                                 Values(av1_highbd_fwd_txfm)));
#endif  // HAVE_AVX2

#if HAVE_NEON && !CONFIG_ADST_TUNED && !CONFIG_INTER_DDT
/* clang-format off */
static TX_SIZE Highbd_fwd_txfm_for_neon[] = {
  TX_4X4,  TX_8X8,  TX_16X16, TX_32X32, TX_64X64, TX_4X8,   TX_8X4,
  TX_8X16, TX_16X8, TX_16X32, TX_32X16, TX_32X64, TX_64X32, TX_4X16,
  TX_16X4, TX_8X32, TX_32X8,  TX_16X64, TX_64X16
};
/* clang-format on */

INSTANTIATE_TEST_SUITE_P(NEON, AV1HighbdFwdTxfm2dTest,
                         Combine(ValuesIn(Highbd_fwd_txfm_for_neon),
                                 Values(av1_highbd_fwd_txfm)));
#endif  // HAVE_NEON && !CONFIG_ADST_TUNED && !CONFIG_INTER_DDT

///////////////////////////////////////////////////////////////
//       unit-test for 'av1_fwd_cross_chroma_tx_block'       //
///////////////////////////////////////////////////////////////

typedef void (*CCTXFunc)(tran_low_t *coeff_c1, tran_low_t *coeff_c2,
                         TX_SIZE tx_size, CctxType cctx_type, const int bd);
typedef libaom_test::FuncParam<CCTXFunc> TestFuncs;

class FwdCctxTest : public ::testing::TestWithParam<TestFuncs> {
 public:
  virtual ~FwdCctxTest() {}
  virtual void SetUp() {
    params_ = this->GetParam();
    rnd_.Reset(ACMRandom::DeterministicSeed());
    const int max_tx_height = tx_size_high[TX_SIZES_LARGEST];
    const int max_tx_width = tx_size_wide[TX_SIZES_LARGEST];
    coeff_alloc_size_ = max_tx_height * max_tx_width * sizeof(coeff_c1_ref_[0]);
    coeff_c1_ref_ =
        reinterpret_cast<int32_t *>(aom_memalign(32, coeff_alloc_size_));
    coeff_c2_ref_ =
        reinterpret_cast<int32_t *>(aom_memalign(32, coeff_alloc_size_));
    coeff_c1_test_ =
        reinterpret_cast<int32_t *>(aom_memalign(32, coeff_alloc_size_));
    coeff_c2_test_ =
        reinterpret_cast<int32_t *>(aom_memalign(32, coeff_alloc_size_));

    ASSERT_TRUE(coeff_c1_ref_ != NULL && coeff_c2_ref_ != NULL);
    ASSERT_TRUE(coeff_c1_test_ != NULL && coeff_c2_test_ != NULL);
  }

  void GenRandomData(const int ncoeffs, const int coeffRange) {
    for (int i = 0; i < ncoeffs; i++) {
      coeff_c1_ref_[i] = rnd_(2) ? rnd_(coeffRange) : -rnd_(coeffRange);
      coeff_c2_ref_[i] = rnd_(2) ? rnd_(coeffRange) : -rnd_(coeffRange);
    }
  }

  void GenExtremeData(const int ncoeffs, const int coeffRange) {
    const int val = rnd_(2) ? coeffRange : -coeffRange;
    for (int i = 0; i < ncoeffs; i++) {
      coeff_c1_ref_[i] = coeff_c2_ref_[i] = val;
    }
  }

  virtual void TearDown() {
    libaom_test::ClearSystemState();
    aom_free(coeff_c1_ref_);
    aom_free(coeff_c2_ref_);
    aom_free(coeff_c1_test_);
    aom_free(coeff_c2_test_);
  }
  void RunTest(int isRandom);
  void RunSpeedTest();

 protected:
  TestFuncs params_;
  tran_low_t *coeff_c1_ref_;
  tran_low_t *coeff_c1_test_;
  tran_low_t *coeff_c2_ref_;
  tran_low_t *coeff_c2_test_;
  int coeff_alloc_size_;
  ACMRandom rnd_;
};

GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(FwdCctxTest);

void FwdCctxTest::RunTest(int isRandom) {
  const int bd_ar[3] = { 8, 10, 12 };
  const int kNumIterations = 500;
  for (int i = 0; i < 3; ++i) {
    const int bd = bd_ar[i];
    const int msb = bd + 8;  // bit-depth + 8
    const int coeffRange = (1 << msb) - 1;
    for (TX_SIZE tx_size = 0; tx_size < TX_SIZES_ALL; tx_size++) {
      const int ncoeffs = av1_get_max_eob(tx_size);
      for (CctxType cctx_type = 0; cctx_type < CCTX_TYPES; cctx_type++) {
        for (int k = 0; k < kNumIterations; k++) {
          if (isRandom) {
            GenRandomData(ncoeffs, coeffRange);
          } else {
            GenExtremeData(ncoeffs, coeffRange);
          }
          memcpy(coeff_c1_test_, coeff_c1_ref_, coeff_alloc_size_);
          memcpy(coeff_c2_test_, coeff_c2_ref_, coeff_alloc_size_);
          params_.ref_func(coeff_c1_ref_, coeff_c2_ref_, tx_size, cctx_type,
                           bd);
          params_.tst_func(coeff_c1_test_, coeff_c2_test_, tx_size, cctx_type,
                           bd);

          for (int i = 0; i < ncoeffs; i++) {
            EXPECT_EQ(coeff_c1_ref_[i], coeff_c1_test_[i])
                << "Error: CCTXTest [tx_size=" << tx_size
                << " cctx_type=" << cctx_type << "] C AVX2 does not match.";
            EXPECT_EQ(coeff_c2_ref_[i], coeff_c2_test_[i])
                << "Error: CCTXTest [tx_size=" << tx_size
                << " cctx_type=" << cctx_type << "] C AVX2 does not match.";
          }
        }
      }
    }
  }
}

void FwdCctxTest::RunSpeedTest() {
  for (TX_SIZE tx_size = 0; tx_size < TX_SIZES_ALL; tx_size++) {
    const CctxType cctx_type = CCTX_45;
    const int ncoeffs = av1_get_max_eob(tx_size);
    const int msb = 12 + 8;  // bit-depth + 8
    const int coeffRange = (1 << msb) - 1;
    GenExtremeData(ncoeffs, coeffRange);

    memcpy(coeff_c1_test_, coeff_c1_ref_, coeff_alloc_size_);
    memcpy(coeff_c2_test_, coeff_c2_ref_, coeff_alloc_size_);

    const int rows = tx_size_high[tx_size];
    const int cols = tx_size_wide[tx_size];
    const int num_loops = 1000000 / (rows * cols);
    aom_usec_timer ref_timer, test_timer;

    aom_usec_timer_start(&ref_timer);
    for (int i = 0; i < num_loops; ++i)
      params_.ref_func(coeff_c1_ref_, coeff_c2_ref_, tx_size, cctx_type, 12);
    aom_usec_timer_mark(&ref_timer);
    const int elapsed_time_c =
        static_cast<int>(aom_usec_timer_elapsed(&ref_timer));

    aom_usec_timer_start(&test_timer);
    for (int i = 0; i < num_loops; ++i)
      params_.tst_func(coeff_c1_test_, coeff_c2_test_, tx_size, cctx_type, 12);
    aom_usec_timer_mark(&test_timer);
    const int elapsed_time_simd =
        static_cast<int>(aom_usec_timer_elapsed(&test_timer));
    printf(
        "txfm_size[%d] \t cctx_type[%d] \t c_time=%d \t simd_time=%d \t "
        "scaling=%0.2f \n",
        tx_size, cctx_type, elapsed_time_c, elapsed_time_simd,
        (float)elapsed_time_c / elapsed_time_simd);
  }
}

TEST_P(FwdCctxTest, BitExactCheck) { RunTest(1); }

TEST_P(FwdCctxTest, ExtremeValues) { RunTest(0); }

TEST_P(FwdCctxTest, DISABLED_Speed) { RunSpeedTest(); }

#if HAVE_AVX2
INSTANTIATE_TEST_SUITE_P(
    AVX2, FwdCctxTest,
    ::testing::Values(TestFuncs(&av1_fwd_cross_chroma_tx_block_c,
                                &av1_fwd_cross_chroma_tx_block_avx2)));
#endif  // HAVE_AVX2

///////////////////////////////////////////////////////////////
//       unit-test for 'fwd_stxfm'                           //
///////////////////////////////////////////////////////////////

typedef void (*FwdSTxfmFunc)(tran_low_t *src, tran_low_t *dst,
                             const PREDICTION_MODE mode, const uint8_t stx_idx,
                             const int size, const int bd);
class AV1FwdSecTxfmTest : public ::testing::TestWithParam<FwdSTxfmFunc> {
 public:
  AV1FwdSecTxfmTest() : fwd_stxfm_func_(GetParam()) {}
  void AV1FwdSecTxfmMatchTest() {
    for (int set_id = 0; set_id < IST_DIR_SIZE; ++set_id) {
      for (uint8_t stx = 0; stx < STX_TYPES - 1; ++stx) {
#if CONFIG_E124_IST_REDUCE_METHOD4
        for (int sb_size = 0; sb_size < 3; ++sb_size) {
#else
        for (int sb_size_idx = 0; sb_size_idx < 2; ++sb_size_idx) {
          int sb_size = (sb_size_idx == 0) ? 4 : 8;
#endif
          DECLARE_ALIGNED(32, tran_low_t, input[IST_8x8_WIDTH]) = { 0 };
          DECLARE_ALIGNED(32, tran_low_t, output[IST_8x8_HEIGHT]) = { 0 };
          DECLARE_ALIGNED(32, tran_low_t, ref_output[IST_8x8_HEIGHT]) = { 0 };
          ACMRandom rnd(ACMRandom::DeterministicSeed());
          const int coeff_range = (1 << (bd + 7)) - 1;
          for (int cnt = 0; cnt < 3; ++cnt) {
            if (cnt == 0) {
              for (int r = 0; r < IST_8x8_WIDTH; ++r) {
                input[r] = coeff_range;
              }
            } else if (cnt == 1) {
              for (int r = 0; r < IST_8x8_WIDTH; ++r) {
                input[r] = -coeff_range;
              }
            } else {
              for (int r = 0; r < IST_8x8_WIDTH; ++r) {
                input[r] = rnd(2) ? rnd(coeff_range) : -rnd(coeff_range);
              }
            }
            fwd_stxfm_c(input, ref_output, set_id, stx, sb_size, bd);
            fwd_stxfm_func_(input, output, set_id, stx, sb_size, bd);
#if CONFIG_E124_IST_REDUCE_METHOD4
            int check_rows = (sb_size == 0)   ? IST_4x4_HEIGHT
                             : (sb_size == 1) ? IST_8x8_HEIGHT_RED
                                              : IST_8x8_HEIGHT;
#else
            int check_rows = (sb_size == 4) ? IST_4x4_HEIGHT : IST_8x8_HEIGHT;
#endif
            for (int r = 0; r < check_rows; ++r) {
              ASSERT_EQ(ref_output[r], output[r])
                  << "[" << r << "] cnt:" << cnt
                  << " ref_output: " << ref_output[r]
                  << " mod_output: " << output[r];
            }
          }
        }
      }
    }
  }

  void AV1FwdSecTxfmSpeedTest() {
#if CONFIG_E124_IST_REDUCE_METHOD4
    for (int sb_size = 0; sb_size < 3; ++sb_size) {
#else
    for (int sb_size_idx = 0; sb_size_idx < 2; ++sb_size_idx) {
      int sb_size = (sb_size_idx == 0) ? 4 : 8;
#endif
      DECLARE_ALIGNED(32, tran_low_t, input[IST_8x8_WIDTH]) = { 0 };
      DECLARE_ALIGNED(32, tran_low_t, output[IST_8x8_HEIGHT]) = { 0 };
      ACMRandom rnd(ACMRandom::DeterministicSeed());
      const int coeff_range = (1 << (bd + 7)) - 1;
      for (int r = 0; r < IST_8x8_WIDTH; ++r) {
        input[r] = rnd(2) ? rnd(coeff_range) : -rnd(coeff_range);
      }
      aom_usec_timer ref_timer, test_timer;
      const int knum_loops = 100000000;
      aom_usec_timer_start(&ref_timer);
      for (int i = 0; i < knum_loops; ++i) {
        fwd_stxfm_c(input, output, 0, 0, sb_size, bd);
      }
      aom_usec_timer_mark(&ref_timer);
      const int elapsed_time_c =
          static_cast<int>(aom_usec_timer_elapsed(&ref_timer));

      aom_usec_timer_start(&test_timer);
      for (int i = 0; i < knum_loops; ++i) {
        fwd_stxfm_func_(input, output, 0, 0, sb_size, bd);
      }
      aom_usec_timer_mark(&test_timer);
      const int elapsed_time_simd =
          static_cast<int>(aom_usec_timer_elapsed(&test_timer));

      printf(
          "sb_size[%d] \t c_time=%d \t simd_time=%d \t "
          "gain=%f \n",
          sb_size, elapsed_time_c, elapsed_time_simd,
          ((1.0 * elapsed_time_c) / elapsed_time_simd));
    }
  }
  FwdSTxfmFunc fwd_stxfm_func_;
};
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AV1FwdSecTxfmTest);

TEST_P(AV1FwdSecTxfmTest, match) { AV1FwdSecTxfmMatchTest(); }
TEST_P(AV1FwdSecTxfmTest, DISABLED_Speed) { AV1FwdSecTxfmSpeedTest(); }

#if HAVE_SSE4_1
INSTANTIATE_TEST_SUITE_P(SSE4_1, AV1FwdSecTxfmTest,
                         ::testing::Values(fwd_stxfm_sse4_1));
#endif  // HAVE_SSE4_1

#if HAVE_AVX2
INSTANTIATE_TEST_SUITE_P(AVX2, AV1FwdSecTxfmTest,
                         ::testing::Values(fwd_stxfm_avx2));
#endif  // HAVE_AVX2

///////////////////////////////////////////////////////////////
//       unit-test for 'inv_stxfm'                           //
///////////////////////////////////////////////////////////////

typedef void (*InvSTxfmFunc)(tran_low_t *src, tran_low_t *dst,
                             const PREDICTION_MODE mode, const uint8_t stx_idx,
                             const int size, const int bd);
class AV1InvSecTxfmTest : public ::testing::TestWithParam<InvSTxfmFunc> {
 public:
  AV1InvSecTxfmTest() : inv_stxfm_func_(GetParam()) {}
  void AV1InvSecTxfmMatchTest() {
    for (int set_id = 0; set_id < IST_DIR_SIZE; ++set_id) {
      for (uint8_t stx = 0; stx < STX_TYPES - 1; ++stx) {
#if CONFIG_E124_IST_REDUCE_METHOD4
        for (int sb_size = 0; sb_size < 3; ++sb_size) {
#else
        for (int sb_size_idx = 0; sb_size_idx < 2; ++sb_size_idx) {
          int sb_size = (sb_size_idx == 0) ? 4 : 8;
#endif
          DECLARE_ALIGNED(32, tran_low_t, input[IST_8x8_HEIGHT]) = { 0 };
          DECLARE_ALIGNED(32, tran_low_t, output[IST_8x8_WIDTH]) = { 0 };
          DECLARE_ALIGNED(32, tran_low_t, ref_output[IST_8x8_WIDTH]) = { 0 };
          ACMRandom rnd(ACMRandom::DeterministicSeed());
          const int coeff_range = ((1 << (bd + 7)) - 1);
          for (int r = 0; r < IST_8x8_HEIGHT; r++) {
            input[r] = rnd(2) ? rnd(coeff_range) : -rnd(coeff_range);
          }
          inv_stxfm_c(input, ref_output, set_id, stx, sb_size, bd);
          inv_stxfm_func_(input, output, set_id, stx, sb_size, bd);
#if CONFIG_E124_IST_REDUCE_METHOD4
          int check_rows = (sb_size == 0) ? IST_4x4_WIDTH : IST_8x8_WIDTH;
#else
          int check_rows = (sb_size == 4) ? IST_4x4_WIDTH : IST_8x8_WIDTH;
#endif
          for (int r = 0; r < check_rows; ++r) {
            ASSERT_EQ(ref_output[r], output[r])
                << "[" << r << "] " << " ref_output: " << ref_output[r]
                << " mod_output: " << output[r];
          }
        }
      }
    }
  }

  void AV1InvSecTxfmSpeedTest() {
#if CONFIG_E124_IST_REDUCE_METHOD4
    for (int sb_size = 0; sb_size < 3; ++sb_size) {
#else
    for (int sb_size_idx = 0; sb_size_idx < 2; ++sb_size_idx) {
      int sb_size = (sb_size_idx == 0) ? 4 : 8;
#endif
      DECLARE_ALIGNED(32, tran_low_t, input[IST_8x8_HEIGHT]) = { 0 };
      DECLARE_ALIGNED(32, tran_low_t, output[IST_8x8_WIDTH]) = { 0 };
      ACMRandom rnd(ACMRandom::DeterministicSeed());
      const int coeff_range = ((1 << (bd + 7)) - 1);
      for (int r = 0; r < IST_8x8_HEIGHT; ++r) {
        input[r] = rnd(2) ? rnd(coeff_range) : -rnd(coeff_range);
      }
      aom_usec_timer ref_timer, test_timer;
      const int knum_loops = 100000000;
      aom_usec_timer_start(&ref_timer);
      for (int i = 0; i < knum_loops; ++i) {
        fwd_stxfm_c(input, output, 0, 0, sb_size, bd);
      }
      aom_usec_timer_mark(&ref_timer);
      const int elapsed_time_c =
          static_cast<int>(aom_usec_timer_elapsed(&ref_timer));

      aom_usec_timer_start(&test_timer);
      for (int i = 0; i < knum_loops; ++i) {
        inv_stxfm_func_(input, output, 0, 0, sb_size, bd);
      }
      aom_usec_timer_mark(&test_timer);
      const int elapsed_time_simd =
          static_cast<int>(aom_usec_timer_elapsed(&test_timer));

      printf(
          "sb_size[%d] \t c_time=%d \t simd_time=%d \t "
          "gain=%f \n",
          sb_size, elapsed_time_c, elapsed_time_simd,
          ((1.0 * elapsed_time_c) / elapsed_time_simd));
    }
  }
  InvSTxfmFunc inv_stxfm_func_;
};

GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(AV1InvSecTxfmTest);

TEST_P(AV1InvSecTxfmTest, match) { AV1InvSecTxfmMatchTest(); }
TEST_P(AV1InvSecTxfmTest, DISABLED_Speed) { AV1InvSecTxfmSpeedTest(); }

#if HAVE_SSE4_1
INSTANTIATE_TEST_SUITE_P(SSE4_1, AV1InvSecTxfmTest,
                         ::testing::Values(inv_stxfm_sse4_1));
#endif  // HAVE_SSE4_1

#if HAVE_AVX2
INSTANTIATE_TEST_SUITE_P(AVX2, AV1InvSecTxfmTest,
                         ::testing::Values(inv_stxfm_avx2));
#endif  // HAVE_AVX2
}  // namespace
