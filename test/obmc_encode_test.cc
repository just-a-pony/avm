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

#include "third_party/googletest/src/googletest/include/gtest/gtest.h"
#include "test/codec_factory.h"
#include "test/encode_test_driver.h"
#include "test/i420_video_source.h"
#include "test/util.h"
#include "test/y4m_video_source.h"

namespace {

class OBMCEncodeTest
    : public ::libaom_test::CodecTestWith2Params<libaom_test::TestMode, int>,
      public ::libaom_test::EncoderTest {
 protected:
  OBMCEncodeTest()
      : EncoderTest(GET_PARAM(0)), encoding_mode_(GET_PARAM(1)),
        set_cpu_used_(GET_PARAM(2)), min_psnr_(DBL_MAX),
        tune_content_(AOM_CONTENT_DEFAULT) {}
  virtual ~OBMCEncodeTest() {}

  virtual void SetUp() {
    InitializeConfig();
    SetMode(encoding_mode_);
    cfg_.g_lag_in_frames = 25;
    cfg_.rc_end_usage = AOM_VBR;
  }

  virtual void BeginPassHook(unsigned int /*pass*/) { min_psnr_ = DBL_MAX; }

  virtual void PreEncodeFrameHook(::libaom_test::VideoSource *video,
                                  ::libaom_test::Encoder *encoder) {
    if (video->frame() == 0) {
      encoder->Control(AOME_SET_CPUUSED, set_cpu_used_);
      encoder->Control(AV1E_SET_TUNE_CONTENT, tune_content_);
      encoder->Control(AOME_SET_ENABLEAUTOALTREF, 1);
      encoder->Control(AOME_SET_ARNR_MAXFRAMES, 7);
      encoder->Control(AOME_SET_ARNR_STRENGTH, 5);
      encoder->Control(AV1E_SET_ENABLE_OBMC, 1);
    }
  }

  virtual void PSNRPktHook(const aom_codec_cx_pkt_t *pkt) {
    if (pkt->data.psnr.psnr[0] < min_psnr_) min_psnr_ = pkt->data.psnr.psnr[0];
  }

  void TestQ0();
  void TestScreencastQ0();
  void TestTuneScreen();
  void TestEncodeHighBitrate();
  void TestLowBitrate();

  ::libaom_test::TestMode encoding_mode_;
  int set_cpu_used_;
  double min_psnr_;
  int tune_content_;
};

void OBMCEncodeTest::TestQ0() {
  // Validate that this non multiple of 64 wide clip encodes and decodes
  // without a mismatch when passing in a very low max q.  This pushes
  // the encoder to producing lots of big partitions which will likely
  // extend into the border and test the border condition.
  cfg_.rc_target_bitrate = 400;
  cfg_.rc_max_quantizer = 0;
  cfg_.rc_min_quantizer = 0;
  const unsigned int width = 208;
  const unsigned int height = 144;
  const unsigned int bit_depth = 8;

  ::libaom_test::I420VideoSource video("hantro_odd.yuv", width, height, 30, 1,
                                       0, 5);

  init_flags_ = AOM_CODEC_USE_PSNR;

  ASSERT_NO_FATAL_FAILURE(RunLoop(&video));
  const double lossless_psnr =
      get_lossless_psnr(width, height, bit_depth, false);
  EXPECT_EQ(min_psnr_, lossless_psnr);
}

void OBMCEncodeTest::TestScreencastQ0() {
  ::libaom_test::Y4mVideoSource video("screendata.y4m", 0, 3);
  cfg_.g_timebase = video.timebase();
  cfg_.rc_target_bitrate = 400;
  cfg_.rc_max_quantizer = 0;
  cfg_.rc_min_quantizer = 0;
  const unsigned int width = 640;
  const unsigned int height = 480;
  const unsigned int bit_depth = 8;

  init_flags_ = AOM_CODEC_USE_PSNR;

  ASSERT_NO_FATAL_FAILURE(RunLoop(&video));

  const double lossless_psnr =
      get_lossless_psnr(width, height, bit_depth, false);
  EXPECT_EQ(min_psnr_, lossless_psnr);
}

void OBMCEncodeTest::TestTuneScreen() {
  ::libaom_test::Y4mVideoSource video("screendata.y4m", 0, 3);
  cfg_.g_timebase = video.timebase();
  cfg_.rc_target_bitrate = 2000;
  cfg_.rc_max_quantizer = 255;
  cfg_.rc_min_quantizer = 0;
  tune_content_ = AOM_CONTENT_SCREEN;

  init_flags_ = AOM_CODEC_USE_PSNR;

  ASSERT_NO_FATAL_FAILURE(RunLoop(&video));
}

void OBMCEncodeTest::TestEncodeHighBitrate() {
  // Validate that this non multiple of 64 wide clip encodes and decodes
  // without a mismatch when passing in a very low max q.  This pushes
  // the encoder to producing lots of big partitions which will likely
  // extend into the border and test the border condition.
  cfg_.rc_target_bitrate = 12000;
  cfg_.rc_max_quantizer = 40;
  cfg_.rc_min_quantizer = 0;

  ::libaom_test::I420VideoSource video("hantro_odd.yuv", 208, 144, 30, 1, 0, 5);

  ASSERT_NO_FATAL_FAILURE(RunLoop(&video));
}

void OBMCEncodeTest::TestLowBitrate() {
  // Validate that this clip encodes and decodes without a mismatch
  // when passing in a very high min q.  This pushes the encoder to producing
  // lots of small partitions which might will test the other condition.
  cfg_.rc_target_bitrate = 200;
  cfg_.rc_min_quantizer = 160;

  ::libaom_test::I420VideoSource video("hantro_odd.yuv", 208, 144, 30, 1, 0,
                                       10);

  ASSERT_NO_FATAL_FAILURE(RunLoop(&video));
}

TEST_P(OBMCEncodeTest, TestQ0) { TestQ0(); }
TEST_P(OBMCEncodeTest, TestScreencastQ0) { TestScreencastQ0(); }
TEST_P(OBMCEncodeTest, TestTuneScreen) { TestTuneScreen(); }
TEST_P(OBMCEncodeTest, TestEncodeHighBitrate) { TestEncodeHighBitrate(); }
TEST_P(OBMCEncodeTest, TestLowBitrate) { TestLowBitrate(); }

class OBMCEncodeTestLarge : public OBMCEncodeTest {};

TEST_P(OBMCEncodeTestLarge, TestQ0) { TestQ0(); }
TEST_P(OBMCEncodeTestLarge, TestScreencastQ0) { TestScreencastQ0(); }
TEST_P(OBMCEncodeTestLarge, TestTuneScreen) { TestTuneScreen(); }
TEST_P(OBMCEncodeTestLarge, TestEncodeHighBitrate) { TestEncodeHighBitrate(); }
TEST_P(OBMCEncodeTestLarge, TestLowBitrate) { TestLowBitrate(); }

AV1_INSTANTIATE_TEST_SUITE(OBMCEncodeTest, GOODQUALITY_TEST_MODES,
                           ::testing::Range(1, 3));
AV1_INSTANTIATE_TEST_SUITE(OBMCEncodeTestLarge, GOODQUALITY_TEST_MODES,
                           ::testing::Range(0, 1));
}  // namespace
