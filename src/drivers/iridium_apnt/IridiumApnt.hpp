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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/Serial.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/iridium_apnt_command.h>
#include <uORB/topics/iridium_apnt_position.h>
#include <uORB/topics/iridium_apnt_pps.h>
#include <uORB/topics/iridium_apnt_response.h>
#include <uORB/topics/iridium_apnt_sentence.h>
#include <uORB/topics/iridium_apnt_skyview.h>
#include <uORB/topics/iridium_apnt_stats.h>
#include <uORB/topics/iridium_apnt_svi.h>
#include <uORB/topics/iridium_apnt_system.h>
#include <uORB/topics/iridium_apnt_velocity.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gnss.h>

#include "apnt_nmea.h"

using namespace time_literals;

/**
 * Driver for the RockBLOCK APNT dev kit (Iridium PNT) over its NMEA serial port.
 *
 * Every decoded sentence is published on its own iridium_apnt_* topic. The position and velocity
 * solution is also published as a sensor_gnss instance, so the estimator can use it like any
 * other GNSS receiver.
 */
class IridiumApnt : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	explicit IridiumApnt(const char *port);
	~IridiumApnt() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	bool init();

private:
	static constexpr uint32_t BAUDRATE = 115200; // fixed by the receiver
	static constexpr hrt_abstime RUN_INTERVAL = 10_ms;
	static constexpr hrt_abstime REOPEN_INTERVAL = 2_s;
	static constexpr hrt_abstime COMMAND_TIMEOUT = 3_s;
	// How long a position waits for the velocity of the same epoch before it is published without
	static constexpr hrt_abstime VELOCITY_WAIT = 250_ms;
	static constexpr hrt_abstime MATCH_WINDOW_US = 500_ms;
	static constexpr int MAX_READS_PER_RUN = 8;

	void Run() override;

	void open_port();
	void read_port();
	void handle_line(char *line, size_t len);
	void handle_sentence(const iridium_apnt::Sentence &s, hrt_abstime now);

	void handle_position(const iridium_apnt::Pos &p, hrt_abstime now);
	void handle_velocity(const iridium_apnt::Vel &v, hrt_abstime now);
	void handle_pps(const iridium_apnt::Pps &p, hrt_abstime now);
	void handle_system(const iridium_apnt::Sys &s, hrt_abstime now);
	void handle_stats(const iridium_apnt::Sta &s, hrt_abstime now);
	void handle_svi(const iridium_apnt::Svi &s, hrt_abstime now);
	void handle_skyview(const iridium_apnt::Ssv &s, hrt_abstime now);
	void handle_response(const iridium_apnt::Rsp &r);
	void handle_parameter_report(const iridium_apnt::Par &p);

	bool velocity_matches_position() const;
	void publish_gnss(hrt_abstime now);

	// Commands
	void process_commands(hrt_abstime now);
	void send_command(const iridium_apnt_command_s &cmd, hrt_abstime now);
	void finish_command(bool ack, bool timeout, int error_code);
	void apply_configuration(hrt_abstime now);

	device::Serial _serial{};
	char _port[32] {};
	uint32_t _device_id{0};
	hrt_abstime _last_open_attempt{0};
	hrt_abstime _last_rx{0};
	bool _was_connected{false};
	bool _config_applied{false};

	iridium_apnt::LineFramer _framer{};

	// Latest decoded state used to build sensor_gnss
	iridium_apnt::Pos _pos{};
	hrt_abstime _pos_rx{0};
	bool _pos_pending{false};
	iridium_apnt::Vel _vel{};
	hrt_abstime _vel_rx{0};
	bool _have_vel{false};
	int _utc_offset_s{iridium_apnt::DEFAULT_GPS_UTC_OFFSET_S};
	uint32_t _system_error{0};
	uint8_t _satellites_in_view{0};

	// $GSSSV group assembly
	iridium_apnt_skyview_s _skyview{};
	int _skyview_next_index{0};

	// The one command waiting for its $GSRSP
	struct {
		bool active{false};
		uint32_t request_id{0};
		hrt_abstime sent{0};
		bool has_par{false};
		iridium_apnt::Par par{};
	} _pending;
	uint32_t _next_internal_id{0x80000000u};

	uORB::PublicationMulti<sensor_gnss_s> _sensor_gnss_pub{ORB_ID(sensor_gnss)};
	uORB::Publication<iridium_apnt_position_s> _position_pub{ORB_ID(iridium_apnt_position)};
	uORB::Publication<iridium_apnt_velocity_s> _velocity_pub{ORB_ID(iridium_apnt_velocity)};
	uORB::Publication<iridium_apnt_pps_s> _pps_pub{ORB_ID(iridium_apnt_pps)};
	uORB::Publication<iridium_apnt_system_s> _system_pub{ORB_ID(iridium_apnt_system)};
	uORB::Publication<iridium_apnt_stats_s> _stats_pub{ORB_ID(iridium_apnt_stats)};
	uORB::Publication<iridium_apnt_svi_s> _svi_pub{ORB_ID(iridium_apnt_svi)};
	uORB::Publication<iridium_apnt_skyview_s> _skyview_pub{ORB_ID(iridium_apnt_skyview)};
	uORB::Publication<iridium_apnt_sentence_s> _sentence_pub{ORB_ID(iridium_apnt_sentence)};
	uORB::Publication<iridium_apnt_response_s> _response_pub{ORB_ID(iridium_apnt_response)};

	uORB::Subscription _command_sub{ORB_ID(iridium_apnt_command)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uint32_t _sentences_ok{0};
	uint32_t _sentences_bad_checksum{0};
	uint32_t _sentences_unknown{0};
	uint32_t _gnss_published{0};
	uint32_t _open_failures{0};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IAPNT_MAX_PFOM>) _param_max_pfom,
		(ParamFloat<px4::params::IAPNT_GEOID_M>) _param_geoid_m,
		(ParamInt<px4::params::IAPNT_KF_MODE>) _param_kf_mode,
		(ParamBool<px4::params::IAPNT_RAW_PUB>) _param_raw_pub
	)
};
