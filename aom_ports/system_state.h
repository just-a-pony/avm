/*
 * Copyright (c) 2021, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 3-Clause Clear License and the
 * Alliance for Open Media Patent License 1.0. If the BSD 3-Clause Clear License was
 * not distributed with this source code in the LICENSE file, you can obtain it
 * at aomedia.org/license/software-license/bsd-3-c-c/.  If the Alliance for Open Media Patent
 * License 1.0 was not distributed with this source code in the PATENTS file, you
 * can obtain it at aomedia.org/license/patent-license/.
 */

#ifndef AOM_AOM_PORTS_SYSTEM_STATE_H_
#define AOM_AOM_PORTS_SYSTEM_STATE_H_

#include "config/aom_config.h"

#if ARCH_X86 || ARCH_X86_64
void aom_reset_mmx_state(void);
#define aom_clear_system_state() aom_reset_mmx_state()
#else
#define aom_clear_system_state()
#endif  // ARCH_X86 || ARCH_X86_64
#endif  // AOM_AOM_PORTS_SYSTEM_STATE_H_
