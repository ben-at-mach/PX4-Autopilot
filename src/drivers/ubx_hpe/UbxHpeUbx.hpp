/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

namespace ubx_hpe
{

// Driver-private UBX configuration keys used for HPE/QC raw capture. Keep these
// local until PX4's generic GPS driver needs the same newer diagnostic messages.
namespace cfg_key
{

static constexpr uint32_t CFG_RATE_MEAS = 0x30210001;
static constexpr uint32_t CFG_RATE_NAV = 0x30210002;
static constexpr uint32_t CFG_RATE_TIMEREF = 0x20210003;

// UBX CFG-MSGOUT keys are ordered by port as I2C, UART1, UART2, USB, then SPI.
// The driver stores I2C keys as the base and derives UART1/UART2 keys by adding
// a port-specific offset, matching PX4's generic u-blox driver convention.
static constexpr uint32_t MSGOUT_UBX_NAV_PVT_I2C = 0x20910006;
static constexpr uint32_t MSGOUT_UBX_NAV_SAT_I2C = 0x20910015;
static constexpr uint32_t MSGOUT_UBX_NAV_STATUS_I2C = 0x2091001a;
static constexpr uint32_t MSGOUT_UBX_NAV_CLOCK_I2C = 0x20910065;
static constexpr uint32_t MSGOUT_UBX_NAV_RELPOSNED_I2C = 0x2091008d;
static constexpr uint32_t MSGOUT_UBX_NAV_SIG_I2C = 0x20910345;

static constexpr uint32_t MSGOUT_UBX_RXM_SFRBX_I2C = 0x20910231;
static constexpr uint32_t MSGOUT_UBX_RXM_RAWX_I2C = 0x209102a4;
static constexpr uint32_t MSGOUT_UBX_RXM_COR_I2C = 0x209106b6;

static constexpr uint32_t MSGOUT_UBX_MON_COMMS_I2C = 0x2091034f;
static constexpr uint32_t MSGOUT_UBX_MON_RF_I2C = 0x20910359;
static constexpr uint32_t MSGOUT_UBX_MON_SPAN_I2C = 0x2091038b;
static constexpr uint32_t MSGOUT_UBX_MON_SYS_I2C = 0x2091069d;

} // namespace cfg_key

} // namespace ubx_hpe
