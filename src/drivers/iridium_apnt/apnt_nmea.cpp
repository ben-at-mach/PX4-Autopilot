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

#include "apnt_nmea.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace iridium_apnt
{

uint8_t checksum(const char *body, size_t len)
{
	uint8_t cs = 0;

	for (size_t i = 0; i < len; i++) {
		cs ^= static_cast<uint8_t>(body[i]);
	}

	return cs;
}

size_t build(char *out, size_t cap, const char *body)
{
	const size_t len = strlen(body);
	// '$' + body + '*' + 2 hex digits + CR LF + NUL
	if (cap < len + 7) {
		return 0;
	}

	return snprintf(out, cap, "$%s*%02X\r\n", body, checksum(body, len));
}

bool LineFramer::feed(uint8_t byte)
{
	if (_ready) {
		_ready = false;
		_len = 0;
	}

	if (byte == '\n') {
		const bool had_data = !_discarding && _len > 0;
		_discarding = false;

		if (had_data && _buf[_len - 1] == '\r') {
			_len--;
		}

		if (!had_data || _len == 0) {
			_len = 0;
			return false;
		}

		_buf[_len] = '\0';
		_ready = true;
		return true;
	}

	if (_discarding) {
		return false;
	}

	if (_len >= MAX_LINE_LENGTH) {
		// No terminator within a full line: wrong baud rate or garbage. Drop it all.
		_discarding = true;
		_len = 0;
		_overflows++;
		return false;
	}

	_buf[_len++] = static_cast<char>(byte);
	return false;
}

bool Sentence::is(const char *name) const
{
	return num_fields > 0 && strcmp(fields[0], name) == 0;
}

const char *Sentence::field(size_t n) const
{
	if (n == 0 || n > num_fields || fields[n - 1][0] == '\0') {
		return nullptr;
	}

	return fields[n - 1];
}

static int hex_value(char c)
{
	if (c >= '0' && c <= '9') { return c - '0'; }

	if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }

	if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }

	return -1;
}

bool parse(char *line, Sentence &s)
{
	s = Sentence{};

	if (line[0] != '$') {
		return false;
	}

	char *star = strrchr(line, '*');

	if (star == nullptr) {
		return false;
	}

	const int hi = hex_value(star[1]);
	const int lo = hi >= 0 ? hex_value(star[2]) : -1;

	if (hi < 0 || lo < 0) {
		return false;
	}

	const size_t body_len = star - (line + 1);
	s.checksum_ok = checksum(line + 1, body_len) == static_cast<uint8_t>(hi << 4 | lo);

	*star = '\0';
	char *p = line + 1;
	s.fields[s.num_fields++] = p;

	for (; *p != '\0'; p++) {
		if (*p == ',' && s.num_fields < MAX_FIELDS) {
			*p = '\0';
			s.fields[s.num_fields++] = p + 1;
		}
	}

	return true;
}

double get_double(const Sentence &s, size_t n)
{
	const double nan = static_cast<double>(NAN);
	const char *str = s.field(n);

	if (str == nullptr) {
		return nan;
	}

	char *end = nullptr;
	const double v = strtod(str, &end);
	return (end != str && *end == '\0') ? v : nan;
}

static int64_t to_int(const char *str, int base, int64_t def)
{
	if (str == nullptr) {
		return def;
	}

	char *end = nullptr;
	const long long v = strtoll(str, &end, base);
	return (end != str && *end == '\0') ? v : def;
}

int64_t get_int(const Sentence &s, size_t n, int64_t def)
{
	return to_int(s.field(n), 10, def);
}

int64_t get_int_auto(const Sentence &s, size_t n, int64_t def)
{
	return to_int(s.field(n), 0, def);
}

uint64_t parse_gps_time_us(const char *str)
{
	if (str == nullptr || str[0] == '\0') {
		return 0;
	}

	uint64_t sec = 0;
	const char *p = str;

	if (*p < '0' || *p > '9') {
		return 0;
	}

	for (; *p >= '0' && *p <= '9'; p++) {
		sec = sec * 10 + (*p - '0');
	}

	uint64_t us = 0;

	if (*p == '.') {
		p++;
		uint64_t scale = 100000;

		for (; *p >= '0' && *p <= '9'; p++) {
			us += (*p - '0') * scale;
			scale /= 10;
		}
	}

	if (*p != '\0') {
		return 0;
	}

	return sec * 1000000ULL + us;
}

void enu_covariance(const double var[3], const double cov[3], float out[9])
{
	const float vE = var[0], vN = var[1], vU = var[2];
	const float cEN = cov[0], cEU = cov[1], cNU = cov[2];
	const float m[9] = {vE, cEN, cEU, cEN, vN, cNU, cEU, cNU, vU};
	memcpy(out, m, sizeof(m));
}

static float getf(const Sentence &s, size_t n)
{
	return static_cast<float>(get_double(s, n));
}

bool decode(const Sentence &s, Pos &o)
{
	if (!s.is("GSPOS")) {
		return false;
	}

	o = Pos{};
	o.gps_time_us = parse_gps_time_us(s.field(3));
	o.lat = get_double(s, 5);
	o.lon = get_double(s, 6);
	o.alt = get_double(s, 7);

	for (size_t i = 0; i < 3; i++) {
		o.ecef[i] = get_double(s, 8 + i);
	}

	const double var[3] = {get_double(s, 11), get_double(s, 12), get_double(s, 13)};
	const double cov[3] = {get_double(s, 14), get_double(s, 15), get_double(s, 16)};
	enu_covariance(var, cov, o.cov);
	o.burst_age = getf(s, 17);
	o.kf_age = getf(s, 19);
	o.pfom = get_int(s, 21);
	o.mode = get_int(s, 22);
	return true;
}

bool decode(const Sentence &s, Vel &o)
{
	if (!s.is("GSVEL")) {
		return false;
	}

	o = Vel{};
	o.gps_time_us = parse_gps_time_us(s.field(3));

	for (size_t i = 0; i < 3; i++) {
		o.vel_enu[i] = getf(s, 4 + i);
	}

	const double var[3] = {get_double(s, 7), get_double(s, 8), get_double(s, 9)};
	const double cov[3] = {get_double(s, 10), get_double(s, 11), get_double(s, 12)};
	enu_covariance(var, cov, o.cov);
	o.burst_age = getf(s, 13);
	o.sensor_age = getf(s, 14);
	o.kf_age = getf(s, 15);
	o.state = get_int(s, 16);
	return true;
}

bool decode(const Sentence &s, Pps &o)
{
	if (!s.is("GSPPS")) {
		return false;
	}

	o = Pps{};
	o.gps_time_s = get_int(s, 3);
	o.utc_offset_s = get_int(s, 4);
	o.leap = get_int(s, 5);
	o.status = get_int(s, 6);
	o.source = get_int(s, 7);
	o.accuracy_ns = getf(s, 8);
	o.tfom = get_int(s, 9);
	o.hard_correction_coming = get_int(s, 10, 0) != 0;
	o.hard_correction_applied = get_int(s, 11, 0) != 0;
	o.iridium_status = get_int(s, 12);
	o.burst_age = getf(s, 13);
	o.iridium_diff_ns = getf(s, 14);
	o.iridium_accuracy_ns = getf(s, 15);
	return true;
}

bool decode(const Sentence &s, Sys &o)
{
	if (!s.is("GSSYS")) {
		return false;
	}

	o = Sys{};
	o.uptime_s = get_int(s, 3, 0);
	o.app_uptime_s = get_int(s, 4, 0);
	o.error = get_int_auto(s, 5);
	o.sub_configured = get_int_auto(s, 6);
	o.sub_enabled = get_int_auto(s, 7);
	o.sub_start_gps_s = get_int(s, 8);
	o.sub_end_gps_s = get_int(s, 9);
	o.ext_status = get_int_auto(s, 11);
	copy_string(o.version, sizeof(o.version), s.field(12));
	return true;
}

bool decode(const Sentence &s, Sta &o)
{
	if (!s.is("GSSTA")) {
		return false;
	}

	o = Sta{};
	o.bursts_total = get_int(s, 7);
	o.bursts_last_minute = get_int(s, 8);
	o.cn0_avg = getf(s, 9);
	o.cn0_max = getf(s, 10);
	o.cn0_min = getf(s, 11);
	return true;
}

bool decode(const Sentence &s, Svi &o)
{
	if (!s.is("GSSVI")) {
		return false;
	}

	o = Svi{};
	o.sv_id = get_int(s, 3);
	o.tov_us = parse_gps_time_us(s.field(4));
	o.toa_s = get_double(s, 5);
	o.burst = get_int(s, 6);
	o.pseudorange = get_double(s, 7);
	o.pseudorange_rate = getf(s, 8);
	o.cn0 = getf(s, 9);
	o.doppler = getf(s, 10);

	for (size_t i = 0; i < 3; i++) {
		o.sv_pos[i] = get_double(s, 11 + i);
		o.sv_vel[i] = getf(s, 14 + i);
	}

	o.state = get_int(s, 17);
	return true;
}

bool decode(const Sentence &s, Ssv &o)
{
	if (!s.is("GSSSV")) {
		return false;
	}

	o = Ssv{};
	o.group_total = get_int(s, 3);
	o.group_index = get_int(s, 4);
	o.sv_count = get_int(s, 5);
	o.sv_id = get_int(s, 6);
	o.elevation = get_int(s, 7);
	o.azimuth = get_int(s, 8);
	o.cn0_min = getf(s, 9);
	o.cn0_max = getf(s, 10);
	return true;
}

bool decode(const Sentence &s, Rsp &o)
{
	if (!s.is("GSRSP")) {
		return false;
	}

	o = Rsp{};
	o.count = get_int(s, 3);
	o.ack = get_int(s, 4, 0) == 1;
	o.error = get_int(s, 5, 0);
	return true;
}

bool decode(const Sentence &s, Par &o)
{
	if (!s.is("GSPAR")) {
		return false;
	}

	o = Par{};
	o.count = get_int(s, 5);
	o.param_id = get_int(s, 3);
	const int64_t n = get_int(s, 6, 0);
	size_t pos = 0;

	for (int64_t i = 0; i < n && 7 + i <= (int64_t)s.num_fields; i++) {
		const char *arg = s.fields[6 + i];
		const size_t len = strlen(arg);

		// Leave room for the separator and the terminating NUL
		if (pos + len + 2 > sizeof(o.args)) {
			break;
		}

		if (i > 0) {
			o.args[pos++] = ',';
		}

		memcpy(o.args + pos, arg, len);
		pos += len;
		o.num_args++;
	}

	o.args[pos] = '\0';
	return true;
}

bool decode(const Sentence &s, Log &o)
{
	if (!s.is("GSLOG")) {
		return false;
	}

	o = Log{};
	copy_string(o.text, sizeof(o.text), s.field(5));
	o.level = get_int(s, 6);
	return true;
}

size_t copy_string(char *dst, size_t cap, const char *src)
{
	if (cap == 0) {
		return 0;
	}

	size_t n = 0;

	if (src != nullptr) {
		n = strnlen(src, cap - 1);
		memcpy(dst, src, n);
	}

	dst[n] = '\0';
	return n;
}

} // namespace iridium_apnt
