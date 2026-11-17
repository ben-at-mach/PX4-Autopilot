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

#include "IridiumApnt.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace iridium_apnt;

ModuleBase::Descriptor IridiumApnt::desc{task_spawn, custom_command, print_usage};

namespace
{

const char *error_name(int code)
{
	switch (code) {
	case 0: return "NONE";

	case -1: return "INVALID_ARG_NB";

	case -2: return "UNSUPPORTED_PARAM_ID";

	case -3: return "UNSUPPORTED_MEMORY_CLASS";

	case -5: return "UNSUPPORTED_REQUEST_ID";

	case -6: return "PARAMETER_VALUE_TOO_LONG";

	case -7: return "PARAMETER_IS_UNSET";

	case -8: return "INVALID_ARG";

	case -9: return "CANNOT_CHANGE_OTP_PARAMETER";

	case -10: return "INVALID_OPERATION";

	case -11: return "UNSUPPORTED_SENTENCE_VERSION";

	default: return "UNKNOWN";
	}
}

/** Number of comma separated arguments in a list, 0 for an empty string. */
unsigned count_args(const char *args)
{
	if (args[0] == '\0') {
		return 0;
	}

	unsigned n = 1;

	for (const char *p = args; *p != '\0'; p++) {
		if (*p == ',') {
			n++;
		}
	}

	return n;
}

uint64_t gps_to_unix_us(uint64_t gps_us, int utc_offset_s)
{
	if (gps_us == 0) {
		return 0;
	}

	return gps_us + GPS_TO_UNIX_EPOCH_S * 1000000ULL - static_cast<uint64_t>(utc_offset_s) * 1000000ULL;
}

} // namespace

IridiumApnt::IridiumApnt(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	strncpy(_port, port, sizeof(_port) - 1);

	// The bus number is the trailing number of the port name, e.g. /dev/ttyS3 -> 3
	size_t end = strlen(_port);
	size_t start = end;

	while (start > 0 && _port[start - 1] >= '0' && _port[start - 1] <= '9') {
		start--;
	}

	device::Device::DeviceId id{};
	id.devid_s.devtype = DRV_GPS_DEVTYPE_IRIDIUM_APNT;
	id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	id.devid_s.bus = (start < end) ? atoi(_port + start) : 0;
	_device_id = id.devid;

	_skyview.device_id = _device_id;
}

IridiumApnt::~IridiumApnt()
{
	ScheduleClear();

	if (_serial.isOpen()) {
		_serial.close();
	}

	perf_free(_loop_perf);
}

bool IridiumApnt::init()
{
	ScheduleOnInterval(RUN_INTERVAL);
	return true;
}

void IridiumApnt::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
	}

	const hrt_abstime now = hrt_absolute_time();

	if (!_serial.isOpen()) {
		if (_was_connected) {
			// Anything half-finished belongs to the old connection
			_was_connected = false;
			_config_applied = false;
			_framer = LineFramer{};
			_skyview_next_index = 0;

			if (_pending.active) {
				finish_command(false, true, 0);
			}
		}

		if (now - _last_open_attempt >= REOPEN_INTERVAL) {
			_last_open_attempt = now;
			open_port();
		}

	} else {
		read_port();
	}

	if (_pos_pending && now - _pos_rx >= VELOCITY_WAIT) {
		publish_gnss(now);
	}

	process_commands(now);
	apply_configuration(now);

	perf_end(_loop_perf);
}

void IridiumApnt::open_port()
{
	if (!_serial.setPort(_port) || !_serial.setBaudrate(BAUDRATE) || !_serial.open()) {
		// Report the first failure only, the retry repeats every few seconds
		if (_open_failures++ == 0) {
			PX4_WARN("cannot open %s, retrying", _port);
		}

		return;
	}

	PX4_INFO("connected %s", _port);
	_open_failures = 0;
	_was_connected = true;
}

void IridiumApnt::read_port()
{
	uint8_t buf[128];

	for (int i = 0; i < MAX_READS_PER_RUN; i++) {
		const ssize_t n = _serial.read(buf, sizeof(buf));

		if (n < 0) {
			// The port is non-blocking, so "no data" is reported as an error
			if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
				return;
			}

			PX4_WARN("read error on %s (%d), reconnecting", _port, errno);
			_serial.close();
			return;
		}

		if (n == 0) {
			return;
		}

		_last_rx = hrt_absolute_time();

		for (ssize_t k = 0; k < n; k++) {
			if (_framer.feed(buf[k])) {
				handle_line(_framer.line(), _framer.length());
			}
		}

		if (static_cast<size_t>(n) < sizeof(buf)) {
			return;
		}
	}
}

void IridiumApnt::handle_line(char *line, size_t len)
{
	const hrt_abstime now = hrt_absolute_time();

	iridium_apnt_sentence_s raw{};

	if (_param_raw_pub.get()) {
		raw.timestamp = now;
		raw.device_id = _device_id;
		raw.len = math::min(len, sizeof(raw.data));
		memcpy(raw.data, line, raw.len);
	}

	Sentence s;
	const bool framed = parse(line, s);

	if (_param_raw_pub.get()) {
		raw.checksum_ok = framed && s.checksum_ok;
		_sentence_pub.publish(raw);
	}

	if (!framed || !s.checksum_ok) {
		_sentences_bad_checksum++;
		return;
	}

	handle_sentence(s, now);
}

void IridiumApnt::handle_sentence(const Sentence &s, hrt_abstime now)
{
	bool known = true;

	if (s.is("GSPOS")) {
		Pos p;
		decode(s, p);
		handle_position(p, now);

	} else if (s.is("GSVEL")) {
		Vel v;
		decode(s, v);
		handle_velocity(v, now);

	} else if (s.is("GSPPS")) {
		Pps p;
		decode(s, p);
		handle_pps(p, now);

	} else if (s.is("GSSYS")) {
		Sys y;
		decode(s, y);
		handle_system(y, now);

	} else if (s.is("GSSTA")) {
		Sta a;
		decode(s, a);
		handle_stats(a, now);

	} else if (s.is("GSSVI")) {
		Svi v;
		decode(s, v);
		handle_svi(v, now);

	} else if (s.is("GSSSV")) {
		Ssv v;
		decode(s, v);
		handle_skyview(v, now);

	} else if (s.is("GSRSP")) {
		Rsp r;
		decode(s, r);
		handle_response(r);

	} else if (s.is("GSPAR")) {
		Par p;
		decode(s, p);
		handle_parameter_report(p);

	} else if (s.is("GSLOG")) {
		Log l;
		decode(s, l);
		PX4_INFO("receiver log[%d]: %s", l.level, l.text);

	} else {
		known = false;
	}

	if (known) {
		_sentences_ok++;

	} else {
		_sentences_unknown++;
	}
}

void IridiumApnt::handle_position(const Pos &p, hrt_abstime now)
{
	iridium_apnt_position_s m{};
	m.timestamp = now;
	m.device_id = _device_id;
	m.gps_time_us = p.gps_time_us;
	m.latitude = p.lat;
	m.longitude = p.lon;
	m.altitude_ellipsoid = p.alt;
	memcpy(m.ecef, p.ecef, sizeof(m.ecef));
	memcpy(m.covariance, p.cov, sizeof(m.covariance));
	m.burst_age = p.burst_age;
	m.kf_age = p.kf_age;
	m.pfom = p.pfom;
	m.mode = p.mode;
	_position_pub.publish(m);

	_pos = p;
	_pos_rx = now;
	_pos_pending = true;

	if (velocity_matches_position()) {
		publish_gnss(now);
	}
}

void IridiumApnt::handle_velocity(const Vel &v, hrt_abstime now)
{
	iridium_apnt_velocity_s m{};
	m.timestamp = now;
	m.device_id = _device_id;
	m.gps_time_us = v.gps_time_us;
	memcpy(m.velocity_enu, v.vel_enu, sizeof(m.velocity_enu));
	memcpy(m.covariance, v.cov, sizeof(m.covariance));
	m.burst_age = v.burst_age;
	m.sensor_age = v.sensor_age;
	m.kf_age = v.kf_age;
	m.state = v.state;
	_velocity_pub.publish(m);

	_vel = v;
	_vel_rx = now;
	_have_vel = true;

	if (_pos_pending && velocity_matches_position()) {
		publish_gnss(now);
	}
}

void IridiumApnt::handle_pps(const Pps &p, hrt_abstime now)
{
	iridium_apnt_pps_s m{};
	m.timestamp = now;
	m.device_id = _device_id;
	m.gps_time_s = p.gps_time_s;
	m.utc_offset_s = p.utc_offset_s;
	m.leap = p.leap;
	m.pps_status = p.status;
	m.source = p.source;
	m.accuracy_ns = p.accuracy_ns;
	m.tfom = p.tfom;
	m.hard_correction_coming = p.hard_correction_coming;
	m.hard_correction_applied = p.hard_correction_applied;
	m.iridium_pps_status = p.iridium_status;
	m.burst_age = p.burst_age;
	m.iridium_diff_ns = p.iridium_diff_ns;
	m.iridium_accuracy_ns = p.iridium_accuracy_ns;
	_pps_pub.publish(m);

	// An empty offset means invalid, so keep the previous value
	if (p.utc_offset_s >= 0) {
		_utc_offset_s = p.utc_offset_s;
	}
}

void IridiumApnt::handle_system(const Sys &s, hrt_abstime now)
{
	iridium_apnt_system_s m{};
	m.timestamp = now;
	m.device_id = _device_id;
	m.uptime_s = s.uptime_s;
	m.app_uptime_s = s.app_uptime_s;
	m.error = s.error;
	m.subscription_configured = s.sub_configured;
	m.subscription_enabled = s.sub_enabled;
	m.subscription_start_gps_s = s.sub_start_gps_s;
	m.subscription_end_gps_s = s.sub_end_gps_s;
	m.extended_status = s.ext_status;
	static_assert(sizeof(m.version) == sizeof(s.version), "version buffers differ");
	memcpy(m.version, s.version, sizeof(m.version));
	_system_pub.publish(m);

	_system_error = s.error;
}

void IridiumApnt::handle_stats(const Sta &s, hrt_abstime now)
{
	iridium_apnt_stats_s m{};
	m.timestamp = now;
	m.device_id = _device_id;
	m.bursts_total = s.bursts_total;
	m.bursts_last_minute = s.bursts_last_minute;
	m.cn0_avg = s.cn0_avg;
	m.cn0_max = s.cn0_max;
	m.cn0_min = s.cn0_min;
	_stats_pub.publish(m);
}

void IridiumApnt::handle_svi(const Svi &s, hrt_abstime now)
{
	iridium_apnt_svi_s m{};
	m.timestamp = now;
	m.device_id = _device_id;
	m.sv_id = s.sv_id;
	m.time_of_validity_us = s.tov_us;
	m.time_of_arrival_s = s.toa_s;
	m.burst_counter = s.burst;
	m.pseudorange = s.pseudorange;
	m.pseudorange_rate = s.pseudorange_rate;
	m.cn0 = s.cn0;
	m.doppler = s.doppler;
	memcpy(m.sv_position_ecef, s.sv_pos, sizeof(m.sv_position_ecef));
	memcpy(m.sv_velocity_ecef, s.sv_vel, sizeof(m.sv_velocity_ecef));
	m.state = s.state;
	_svi_pub.publish(m);
}

void IridiumApnt::handle_skyview(const Ssv &s, hrt_abstime now)
{
	if (s.group_index == 1) {
		_skyview = iridium_apnt_skyview_s{};
		_skyview.device_id = _device_id;
		_skyview.satellites_in_view = math::constrain(s.sv_count, 0, 255);
		_skyview_next_index = 1;
	}

	// A sentence of the group was lost or the group started before we connected: drop it whole
	if (s.group_index != _skyview_next_index) {
		_skyview_next_index = 0;
		return;
	}

	_skyview_next_index++;

	const uint8_t i = _skyview.satellite_count;

	if (i < iridium_apnt_skyview_s::MAX_SATELLITES) {
		_skyview.sv_id[i] = s.sv_id;
		_skyview.elevation[i] = s.elevation;
		_skyview.azimuth[i] = s.azimuth;
		_skyview.cn0_min[i] = s.cn0_min;
		_skyview.cn0_max[i] = s.cn0_max;
		_skyview.satellite_count++;
	}

	if (s.group_index == s.group_total) {
		_skyview.timestamp = now;
		_skyview_pub.publish(_skyview);
		_satellites_in_view = _skyview.satellites_in_view;
		_skyview_next_index = 0;
	}
}

bool IridiumApnt::velocity_matches_position() const
{
	if (!_have_vel) {
		return false;
	}

	if (_pos.gps_time_us != 0 && _vel.gps_time_us != 0) {
		const int64_t dt = static_cast<int64_t>(_pos.gps_time_us) - static_cast<int64_t>(_vel.gps_time_us);
		return llabs(dt) <= static_cast<int64_t>(MATCH_WINDOW_US);
	}

	// Without receiver time, fall back on arrival time, as both sentences come out together
	const int64_t dt = static_cast<int64_t>(_pos_rx) - static_cast<int64_t>(_vel_rx);
	return llabs(dt) <= static_cast<int64_t>(VELOCITY_WAIT);
}

void IridiumApnt::publish_gnss(hrt_abstime now)
{
	_pos_pending = false;

	const bool finite_position = PX4_ISFINITE(_pos.lat) && PX4_ISFINITE(_pos.lon) && PX4_ISFINITE(_pos.alt)
				     && fabs(_pos.lat) <= 90. && fabs(_pos.lon) <= 180.;
	const bool pfom_ok = _pos.pfom < 0 || _pos.pfom <= _param_max_pfom.get();
	const bool has_fix = finite_position && pfom_ok;

	sensor_gnss_s g{};
	g.timestamp = now;
	g.timestamp_sample = _pos_rx;
	g.device_id = _device_id;

	g.hdop = 99.99f;
	g.vdop = 99.99f;
	g.eph = 99.f;
	g.epv = 99.f;
	g.speed_accuracy = 99.f;
	g.course_accuracy = M_PI_F;

	g.fix_type = has_fix ? sensor_gnss_s::FIX_TYPE_3D : sensor_gnss_s::FIX_TYPE_NONE;

	if (finite_position) {
		g.latitude = _pos.lat;
		g.longitude = _pos.lon;
		g.altitude_ellipsoid = _pos.alt;
		g.altitude_msl = _pos.alt - static_cast<double>(_param_geoid_m.get());
	}

	// Covariance is row-major East North Up
	const float var_e = _pos.cov[0];
	const float var_n = _pos.cov[4];
	const float var_u = _pos.cov[8];

	if (PX4_ISFINITE(var_e) && PX4_ISFINITE(var_n) && var_e >= 0.f && var_n >= 0.f) {
		g.eph = sqrtf(var_e + var_n);
	}

	if (PX4_ISFINITE(var_u) && var_u >= 0.f) {
		g.epv = sqrtf(var_u);
	}

	const bool vel_ok = velocity_matches_position() && _vel.state == 3
			    && PX4_ISFINITE(_vel.vel_enu[0]) && PX4_ISFINITE(_vel.vel_enu[1]) && PX4_ISFINITE(_vel.vel_enu[2]);

	if (vel_ok) {
		const float ve = _vel.vel_enu[0];
		const float vn = _vel.vel_enu[1];
		const float vu = _vel.vel_enu[2];
		g.vel_east = ve;
		g.vel_north = vn;
		g.vel_down = -vu;
		g.ground_speed = sqrtf(ve * ve + vn * vn);
		g.course = atan2f(ve, vn);
		g.vel_ned_valid = true;

		const float v_var_e = _vel.cov[0];
		const float v_var_n = _vel.cov[4];
		const float v_var_u = _vel.cov[8];

		if (PX4_ISFINITE(v_var_e) && PX4_ISFINITE(v_var_n) && PX4_ISFINITE(v_var_u)
		    && v_var_e >= 0.f && v_var_n >= 0.f && v_var_u >= 0.f) {
			g.speed_accuracy = sqrtf(v_var_e + v_var_n + v_var_u);

			if (g.ground_speed > 0.5f) {
				g.course_accuracy = math::min(sqrtf(v_var_e + v_var_n) / g.ground_speed, M_PI_F);
			}
		}
	}

	g.time_utc_usec = gps_to_unix_us(_pos.gps_time_us, _utc_offset_s);
	g.timestamp_time_relative = 0;

	// The sentence does not say how many satellites went into the solution, only how many are visible
	g.satellites_used = _satellites_in_view;

	// Error bits 0..4 are missing identity, subscription or key. Bits 5 and 6 are the normal start-up state.
	if (_system_error & 0x1f) {
		g.system_error |= sensor_gnss_s::SYSTEM_ERROR_CONFIGURATION;
	}

	_sensor_gnss_pub.publish(g);
	_gnss_published++;
}

void IridiumApnt::handle_response(const Rsp &r)
{
	if (_pending.active && static_cast<uint32_t>(r.count) == _pending.request_id) {
		finish_command(r.ack, false, r.error);
	}
}

void IridiumApnt::handle_parameter_report(const Par &p)
{
	if (_pending.active && static_cast<uint32_t>(p.count) == _pending.request_id) {
		_pending.has_par = true;
		_pending.par = p;
	}
}

void IridiumApnt::process_commands(hrt_abstime now)
{
	if (_pending.active) {
		if (now - _pending.sent >= COMMAND_TIMEOUT) {
			finish_command(false, true, 0);
		}

		return;
	}

	iridium_apnt_command_s cmd;

	if (_command_sub.update(&cmd)) {
		send_command(cmd, now);
	}
}

void IridiumApnt::send_command(const iridium_apnt_command_s &cmd, hrt_abstime now)
{
	_pending = {};
	_pending.active = true;
	_pending.request_id = cmd.request_id;
	_pending.sent = now;

	char args[sizeof(cmd.args)];
	memcpy(args, cmd.args, sizeof(args));
	args[sizeof(args) - 1] = '\0';

	char body[160];
	const int mem = cmd.nvm ? 2 : 1;
	int len = 0;

	switch (cmd.type) {
	case iridium_apnt_command_s::TYPE_GET:
		len = snprintf(body, sizeof(body), "GSGET,%d,%u,%d,%lu", SENTENCE_VERSION, cmd.param_id, mem,
			       static_cast<unsigned long>(cmd.request_id));
		break;

	case iridium_apnt_command_s::TYPE_SET:
		len = snprintf(body, sizeof(body), "GSSET,%d,%u,%d,%lu,%u%s%s", SENTENCE_VERSION, cmd.param_id, mem,
			       static_cast<unsigned long>(cmd.request_id), count_args(args), args[0] ? "," : "", args);
		break;

	case iridium_apnt_command_s::TYPE_REQUEST:
		len = snprintf(body, sizeof(body), "GSREQ,%d,%u,%lu,%u%s%s", SENTENCE_VERSION, cmd.param_id,
			       static_cast<unsigned long>(cmd.request_id), count_args(args), args[0] ? "," : "", args);
		break;

	default:
		finish_command(false, false, -10);
		return;
	}

	char frame[sizeof(body) + 8];
	const size_t frame_len = (len > 0 && len < static_cast<int>(sizeof(body))) ? build(frame, sizeof(frame), body) : 0;

	if (frame_len == 0 || !_serial.isOpen() || _serial.write(frame, frame_len) != static_cast<ssize_t>(frame_len)) {
		finish_command(false, true, 0);
	}
}

void IridiumApnt::finish_command(bool ack, bool timeout, int error_code)
{
	iridium_apnt_response_s r{};
	r.timestamp = hrt_absolute_time();
	r.request_id = _pending.request_id;
	r.ack = ack;
	r.timeout = timeout;
	r.error_code = error_code;

	if (ack && _pending.has_par) {
		r.param_id = _pending.par.param_id;
		static_assert(sizeof(r.args) == sizeof(_pending.par.args), "argument buffers differ");
		memcpy(r.args, _pending.par.args, sizeof(r.args));
	}

	_response_pub.publish(r);
	_pending = {};
}

void IridiumApnt::apply_configuration(hrt_abstime now)
{
	// Wait for the first sentence so the receiver is known to be listening
	if (_config_applied || _pending.active || !_serial.isOpen() || _last_rx == 0) {
		return;
	}

	_config_applied = true;
	const int mode = _param_kf_mode.get();

	if (mode < 0) {
		return;
	}

	iridium_apnt_command_s cmd{};
	cmd.request_id = _next_internal_id++;
	cmd.type = iridium_apnt_command_s::TYPE_SET;
	cmd.param_id = 101;
	cmd.nvm = false;
	snprintf(reinterpret_cast<char *>(cmd.args), sizeof(cmd.args), "%d", mode);
	send_command(cmd, now);
}

int IridiumApnt::print_status()
{
	const hrt_abstime now = hrt_absolute_time();

	PX4_INFO("port %s: %s", _port, _serial.isOpen() ? "connected" : "NOT CONNECTED");
	PX4_INFO("sentences ok %lu, bad checksum %lu, unknown %lu, long lines dropped %lu",
		 static_cast<unsigned long>(_sentences_ok), static_cast<unsigned long>(_sentences_bad_checksum),
		 static_cast<unsigned long>(_sentences_unknown), static_cast<unsigned long>(_framer.overflows()));

	if (_last_rx != 0) {
		PX4_INFO("last data %.1f s ago", (double)(now - _last_rx) * 1e-6);
	}

	PX4_INFO("sensor_gnss published %lu", static_cast<unsigned long>(_gnss_published));

	if (_pos_rx != 0) {
		PX4_INFO("position %.7f %.7f %.1f m (ellipsoid), pfom %d, mode %d, %.1f s ago",
			 _pos.lat, _pos.lon, _pos.alt, _pos.pfom, _pos.mode, (double)(now - _pos_rx) * 1e-6);
	}

	if (_have_vel) {
		PX4_INFO("velocity state %d, %.1f s ago", _vel.state, (double)(now - _vel_rx) * 1e-6);
	}

	PX4_INFO("receiver error bits 0x%02lx, satellites in view %u", static_cast<unsigned long>(_system_error),
		 _satellites_in_view);

	perf_print_counter(_loop_perf);
	return 0;
}

int IridiumApnt::task_spawn(int argc, char *argv[])
{
	const char *port = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			port = myoptarg;
			break;

		default:
			print_usage("unrecognized option");
			return PX4_ERROR;
		}
	}

	if (port == nullptr || port[0] == '\0') {
		print_usage("no serial device given");
		return PX4_ERROR;
	}

	IridiumApnt *instance = new IridiumApnt(port);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

namespace
{

/** Send a command through the running driver and print its answer. */
int run_command(uint8_t type, unsigned param_id, bool nvm, const char *args)
{
	if (!ModuleBase::is_running(IridiumApnt::desc)) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	// Subscribe first so the answer cannot be missed
	uORB::Subscription response_sub{ORB_ID(iridium_apnt_response)};
	uORB::Publication<iridium_apnt_command_s> command_pub{ORB_ID(iridium_apnt_command)};

	iridium_apnt_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	// Internal requests count up from 0x80000000, so stay below it
	cmd.request_id = static_cast<uint32_t>(cmd.timestamp / 1000) & 0x3fffffff;
	cmd.type = type;
	cmd.param_id = param_id;
	cmd.nvm = nvm;
	snprintf(reinterpret_cast<char *>(cmd.args), sizeof(cmd.args), "%s", args);
	command_pub.publish(cmd);

	const hrt_abstime deadline = hrt_absolute_time() + 4_s;

	while (hrt_absolute_time() < deadline) {
		iridium_apnt_response_s r;

		while (response_sub.update(&r)) {
			if (r.request_id != cmd.request_id) {
				continue;
			}

			if (r.timeout) {
				PX4_ERR("no answer from the receiver");
				return PX4_ERROR;
			}

			if (!r.ack) {
				PX4_ERR("NACK: %s (%d)", error_name(r.error_code), r.error_code);
				return PX4_ERROR;
			}

			if (type == iridium_apnt_command_s::TYPE_GET) {
				PX4_INFO("param %u = %s", r.param_id, reinterpret_cast<const char *>(r.args));

			} else {
				PX4_INFO("ACK");
			}

			return PX4_OK;
		}

		px4_usleep(20000);
	}

	PX4_ERR("driver did not answer");
	return PX4_ERROR;
}

} // namespace

int IridiumApnt::custom_command(int argc, char *argv[])
{
	// Collect the words, taking "-n" (non-volatile memory) out of them
	const char *words[8];
	int nwords = 0;
	bool nvm = false;

	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "-n") == 0) {
			nvm = true;

		} else if (nwords < 8) {
			words[nwords++] = argv[i];
		}
	}

	if (nwords == 0) {
		return print_usage("unknown command");
	}

	const char *cmd = words[0];
	char args[96] {};

	auto join = [&](int first) {
		size_t pos = 0;

		for (int i = first; i < nwords; i++) {
			const int n = snprintf(args + pos, sizeof(args) - pos, "%s%s", i > first ? "," : "", words[i]);

			if (n < 0 || pos + n >= sizeof(args)) {
				return false;
			}

			pos += n;
		}

		return true;
	};

	if (strcmp(cmd, "get") == 0 && nwords == 2) {
		return run_command(iridium_apnt_command_s::TYPE_GET, atoi(words[1]), nvm, "");

	} else if (strcmp(cmd, "set") == 0 && nwords >= 3) {
		if (!join(2)) {
			PX4_ERR("arguments too long");
			return PX4_ERROR;
		}

		return run_command(iridium_apnt_command_s::TYPE_SET, atoi(words[1]), nvm, args);

	} else if (strcmp(cmd, "request") == 0 && nwords >= 2) {
		if (!join(2)) {
			PX4_ERR("arguments too long");
			return PX4_ERROR;
		}

		return run_command(iridium_apnt_command_s::TYPE_REQUEST, atoi(words[1]), nvm, args);

	} else if (strcmp(cmd, "kalman") == 0 && nwords == 2) {
		const char *mode = nullptr;

		if (strcmp(words[1], "surveyed") == 0) { mode = "0"; }

		else if (strcmp(words[1], "static") == 0) { mode = "1"; }

		else if (strcmp(words[1], "dynamic") == 0) { mode = "3"; }

		if (mode == nullptr) {
			return print_usage("kalman mode must be surveyed, static or dynamic");
		}

		return run_command(iridium_apnt_command_s::TYPE_SET, 101, nvm, mode);

	} else if (strcmp(cmd, "dynamic_model") == 0 && nwords == 5) {
		if (!join(1)) {
			PX4_ERR("arguments too long");
			return PX4_ERROR;
		}

		return run_command(iridium_apnt_command_s::TYPE_SET, 102, nvm, args);

	} else if (strcmp(cmd, "reboot") == 0 && nwords == 1) {
		return run_command(iridium_apnt_command_s::TYPE_REQUEST, 1, false, "4");
	}

	return print_usage("unknown command");
}

int IridiumApnt::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for the RockBLOCK APNT dev kit, which outputs position, velocity and timing from the
Iridium PNT service as NMEA over a serial port (115200 8N1, fixed).

Every sentence is published on its own `iridium_apnt_*` topic. Position and velocity are also
published as a `sensor_gnss` instance, so the estimator can fuse them like a GNSS receiver.
Raw sentences are published on `iridium_apnt_sentence` when `IAPNT_RAW_PUB` is set.

Receiver parameters can be read and written with the commands below, or by publishing
`iridium_apnt_command` and waiting for `iridium_apnt_response`. A command is answered within 3 s.
By default it uses volatile memory (current session). With `-n` it uses non-volatile memory.

### Examples

Start on a serial port:
$ iridium_apnt start -d /dev/ttyS3

Read the Kalman filter mode from non-volatile memory:
$ iridium_apnt get 101 -n

Set the Kalman filter to dynamic mode and use a fixed-wing dynamic model:
$ iridium_apnt kalman dynamic
$ iridium_apnt dynamic_model 30.0 300.0 10.0 100.0
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("iridium_apnt", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("gnss");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("get", "Read a receiver parameter ($GSGET)");
	PRINT_MODULE_USAGE_ARG("<param_id>", "Receiver parameter ID", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Write a receiver parameter ($GSSET)");
	PRINT_MODULE_USAGE_ARG("<param_id> <arg>...", "Receiver parameter ID and its arguments", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("request", "Run a receiver request ($GSREQ)");
	PRINT_MODULE_USAGE_ARG("<request_id> [<arg>...]", "1 reset, 5 clear NVM parameter, 6 clear all NVM parameters", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("kalman", "Set the Kalman filter mode (parameter 101)");
	PRINT_MODULE_USAGE_ARG("surveyed|static|dynamic", "Filter mode", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("dynamic_model", "Set the dynamic model (parameter 102)");
	PRINT_MODULE_USAGE_ARG("<vtc> <htc> <vvstd> <hvstd>", "Time constants and velocity std", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reboot", "Reboot the receiver");
	PRINT_MODULE_USAGE_PARAM_FLAG('n', "Use non-volatile memory for get, set, kalman and dynamic_model", true);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");

	return PX4_OK;
}

extern "C" __EXPORT int iridium_apnt_main(int argc, char *argv[])
{
	return ModuleBase::main(IridiumApnt::desc, argc, argv);
}
