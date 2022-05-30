/*
 * Copyright (C) 2018 ETH Zurich and University of Bologna and
 * GreenWaves Technologies
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __RT_RT_API_DECL_H__
#define __RT_RT_API_DECL_H__

#include "rt_data.h"

#include "chips/rt_default.h"
#include "chips/rt_gap.h"
#include "chips/rt_wolfe.h"
#include "chips/rt_vega.h"
#include "chips/rt_gap9.h"
#include "chips/rt_pulpissimo.h"
#include "chips/rt_pulp.h"

#include "rt_irq.h"
#include "rt_utils.h"
#include "rt_extern_alloc.h"
#include "rt_thread.h"
#include "rt_event.h"
#include "rt_flash.h"
#include "rt_dev.h"
#include "rt_periph.h"
#include "rt_cluster.h"
#include "rt_hyper.h"
#include "rt_alloc.h"
#include "rt_debug.h"
#include "rt_config.h"
#include "rt_pe.h"
#include "rt_i2c.h"
#include "rt_camera.h"
#include "rt_himax.h"
#include "rt_ov7670.h"
#include "rt_dma.h"
#include "rt_sync_mc.h"
#include "rt_perf.h"
#include "rt_time.h"
#include "rt_timer.h"
#include "rt_freq.h"
#include "rt_pm.h"
#include "rt_i2s.h"
#include "rt_fs.h"
#include "rt_error.h"
#if defined(ARCHI_UDMA_HAS_UART) && UDMA_VERSION >= 2 || defined(ARCHI_HAS_UART)
#include "rt_uart.h"
#endif
#include "rt_spim.h"
#include "rt_rtc.h"
#if PULP_CHIP_FAMILY == CHIP_GAP
#include "rt_pwm.h"
#endif
#if defined(APB_SOC_VERSION) && APB_SOC_VERSION == 1
#include "rt_pad.h"
#endif
#ifdef GPIO_VERSION
#include "rt_gpio.h"
#endif
#include "rt_voltage.h"
#include "rt_bridge.h"
#include "rt_eeprom.h"
#include "rt_task.h"

#endif
