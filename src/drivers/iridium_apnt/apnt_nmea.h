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

/**
 * @file apnt_nmea.h
 *
 * NMEA framing and decoding for the RockBLOCK APNT (Iridium PNT) receiver.
 *
 * Sentences look like "$GSPOS,3,...*CS\r\n" with an 8-bit XOR checksum over everything between
 * '$' and '*'. Field numbers in the vendor documentation are 1-based and count the header, so
 * the decoders here use the same numbers: field 1 is the header, field 3 is the first data field.
 *
 * This file has no PX4 dependencies so it can be unit tested on the host.
 */

#include <stddef.h>
#include <stdint.h>

namespace iridium_apnt
{

static constexpr size_t MAX_LINE_LENGTH = 256;
static constexpr size_t MAX_FIELDS = 40;
static constexpr size_t MAX_SSV = 24;
static constexpr size_t MAX_ARGS_LENGTH = 96;

// Sentence version understood by the commands we send.
static constexpr int SENTENCE_VERSION = 2;

static constexpr uint64_t GPS_TO_UNIX_EPOCH_S = 315964800ULL; // 1980-01-06 minus 1970-01-01
static constexpr int DEFAULT_GPS_UTC_OFFSET_S = 18;

/** XOR of all characters between '$' and '*' (exclusive). */
uint8_t checksum(const char *body, size_t len);

/**
 * Frame a sentence body ("GSGET,2,101,2,6") as "$body*CS\r\n".
 * @return length written without the terminating NUL, or 0 if cap is too small.
 */
size_t build(char *out, size_t cap, const char *body);

/**
 * Collects bytes into lines. A line ends at '\n'; a trailing '\r' is dropped. A line longer than
 * MAX_LINE_LENGTH is discarded in full and counted in overflows().
 */
class LineFramer
{
public:
	/** Feed one byte. @return true when a complete, non-empty line is available. */
	bool feed(uint8_t byte);

	/** The completed line, NUL terminated. Valid until the next feed(). */
	char *line() { return _buf; }
	size_t length() const { return _len; }
	uint32_t overflows() const { return _overflows; }

private:
	char _buf[MAX_LINE_LENGTH + 1] {};
	size_t _len{0};
	bool _discarding{false};
	bool _ready{false};
	uint32_t _overflows{0};
};

/**
 * A sentence split in place. parse() overwrites the commas of the line with NUL, so the line
 * buffer must outlive the Sentence.
 */
struct Sentence {
	bool checksum_ok{false};
	size_t num_fields{0};
	const char *fields[MAX_FIELDS] {};

	/** Header such as "GSPOS", or "" if the line is not a sentence. */
	const char *header() const { return num_fields > 0 ? fields[0] : ""; }
	bool is(const char *name) const;

	/** Doc field number n (1-based) or nullptr if missing or empty. */
	const char *field(size_t n) const;
};

/**
 * Split a line. Returns false (and leaves s empty) if the line does not start with '$' or
 * has no "*CS" trailer. A bad checksum still returns true with checksum_ok == false.
 */
bool parse(char *line, Sentence &s);

// Empty or unparsable fields decode to NaN for floats and to the given default for integers.
double get_double(const Sentence &s, size_t n);
int64_t get_int(const Sentence &s, size_t n, int64_t def = -1);
/** Integer in any base C understands, e.g. "0x40". */
int64_t get_int_auto(const Sentence &s, size_t n, int64_t def = 0);

/**
 * "seconds.fraction" since the GPS epoch -> microseconds. Parsed as integers because the
 * receiver sends 8 or more fractional digits, which a double cannot hold at 1e9 seconds.
 * @return 0 if the string is missing or malformed.
 */
uint64_t parse_gps_time_us(const char *str);

/** Row-major East-North-Up 3x3 from (vE, vN, vU) and (cEN, cEU, cNU). */
void enu_covariance(const double var[3], const double cov[3], float out[9]);

// Decoded sentences. Unknown floats are NaN and unknown coded values are -1.
struct Pos {
	uint64_t gps_time_us;
	double lat, lon, alt;
	double ecef[3];
	float cov[9];
	float burst_age, kf_age;
	int pfom, mode;
};

struct Vel {
	uint64_t gps_time_us;
	float vel_enu[3];
	float cov[9];
	float burst_age, sensor_age, kf_age;
	int state;
};

struct Pps {
	int64_t gps_time_s;
	int utc_offset_s, leap, status, source;
	float accuracy_ns;
	int tfom;
	bool hard_correction_coming, hard_correction_applied;
	int iridium_status;
	float burst_age, iridium_diff_ns, iridium_accuracy_ns;
};

struct Sys {
	uint64_t uptime_s, app_uptime_s;
	uint32_t error, sub_configured, sub_enabled, ext_status;
	int64_t sub_start_gps_s, sub_end_gps_s;
	char version[24];
};

struct Sta {
	int64_t bursts_total;
	int bursts_last_minute;
	float cn0_avg, cn0_max, cn0_min;
};

struct Svi {
	int sv_id;
	uint64_t tov_us;
	double toa_s;
	int64_t burst;
	double pseudorange;
	float pseudorange_rate, cn0, doppler;
	double sv_pos[3];
	float sv_vel[3];
	int state;
};

/** One $GSSSV sentence: one satellite of a group. */
struct Ssv {
	int group_total, group_index, sv_count;
	int sv_id, elevation, azimuth;
	float cn0_min, cn0_max;
};

struct Rsp {
	int64_t count;
	bool ack;
	int error;
};

struct Par {
	int64_t count;
	int param_id;
	size_t num_args;
	/** Arguments joined with ',' and NUL terminated. */
	char args[MAX_ARGS_LENGTH];
};

bool decode(const Sentence &s, Pos &out);
bool decode(const Sentence &s, Vel &out);
bool decode(const Sentence &s, Pps &out);
bool decode(const Sentence &s, Sys &out);
bool decode(const Sentence &s, Sta &out);
bool decode(const Sentence &s, Svi &out);
bool decode(const Sentence &s, Ssv &out);
bool decode(const Sentence &s, Rsp &out);
bool decode(const Sentence &s, Par &out);

struct Log {
	int level;
	char text[96];
};

bool decode(const Sentence &s, Log &out);

/** Copy at most cap - 1 characters and NUL terminate. @return characters copied. */
size_t copy_string(char *dst, size_t cap, const char *src);

} // namespace iridium_apnt
