/*******************************************************************************
 * Copyright 2021 MINRES Technologies GmbH
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
 *******************************************************************************/

#pragma once

/**
 * @defgroup scc-common Common utilities
 *
 * This module contains generic C++ functions being independent of SystemC
 */
/**@{*/
#include <util/sccassert.h>
#include "util/bit_field.h"
#include "util/delegate.h"
#include "util/io-redirector.h"
#include "util/ities.h"
#include "util/logging.h"
#include "util/mt19937_rng.h"
#include "util/pool_allocator.h"
#include "util/range_lut.h"
#include "util/sparse_array.h"
#include "util/strprintf.h"
#include "util/thread_syncronizer.h"
#include "util/watchdog.h"
/**@}*/
