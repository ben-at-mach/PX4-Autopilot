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

#include <gtest/gtest.h>

#include <math.h>
#include <string.h>

#include "apnt_nmea.h"

using namespace iridium_apnt;

namespace
{

// Parse a sentence given without framing, adding the checksum.
struct Parsed {
	char line[MAX_LINE_LENGTH + 1];
	Sentence s;
	bool framed;

	explicit Parsed(const char *body)
	{
		char frame[MAX_LINE_LENGTH + 8];
		build(frame, sizeof(frame), body);
		strncpy(line, frame, sizeof(line) - 1);
		line[sizeof(line) - 1] = '\0';
		// The framer would have stripped the line ending
		line[strcspn(line, "\r\n")] = '\0';
		framed = parse(line, s);
	}
};

} // namespace

TEST(ApntNmea, VendorExampleChecksum)
{
	// Published in the vendor documentation
	char out[64];
	ASSERT_GT(build(out, sizeof(out), "GSSET,2,101,2,4,1,1"), 0u);
	EXPECT_STREQ(out, "$GSSET,2,101,2,4,1,1*52\r\n");

	ASSERT_GT(build(out, sizeof(out), "GSGET,2,101,2,6"), 0u);
	EXPECT_STREQ(out, "$GSGET,2,101,2,6*44\r\n");

	ASSERT_GT(build(out, sizeof(out), "GSREQ,2,1,5,1,4"), 0u);
	EXPECT_EQ(strncmp(out, "$GSREQ,2,1,5,1,4*", 17), 0);
}

TEST(ApntNmea, BuildRefusesSmallBuffer)
{
	char out[8];
	EXPECT_EQ(build(out, sizeof(out), "GSGET,2,101,2,6"), 0u);
}

TEST(ApntNmea, FramerSplitsLinesAndDropsCr)
{
	LineFramer f;
	const char *data = "$A*00\r\n\r\n$B*00\n$C";
	int lines = 0;
	char first[16] = {};

	for (const char *p = data; *p; p++) {
		if (f.feed(*p)) {
			if (lines == 0) { strncpy(first, f.line(), sizeof(first) - 1); }

			lines++;
		}
	}

	EXPECT_EQ(lines, 2); // the empty line and the unfinished "$C" do not count
	EXPECT_STREQ(first, "$A*00");
}

TEST(ApntNmea, FramerDropsOverlongLineWholeAndRecovers)
{
	LineFramer f;

	for (size_t i = 0; i < MAX_LINE_LENGTH + 50; i++) {
		EXPECT_FALSE(f.feed('x'));
	}

	EXPECT_FALSE(f.feed('\n')); // the tail of the long line must not come out as a line
	EXPECT_EQ(f.overflows(), 1u);

	bool got = false;

	for (const char *p = "$OK*00\r\n"; *p; p++) {
		got = f.feed(*p);
	}

	EXPECT_TRUE(got);
	EXPECT_STREQ(f.line(), "$OK*00");
}

TEST(ApntNmea, ChecksumVerification)
{
	char good[] = "$GSSET,2,101,2,4,1,1*52";
	Sentence s;
	EXPECT_TRUE(parse(good, s));
	EXPECT_TRUE(s.checksum_ok);
	EXPECT_TRUE(s.is("GSSET"));

	char bad[] = "$GSSET,2,101,2,4,1,1*53";
	EXPECT_TRUE(parse(bad, s));
	EXPECT_FALSE(s.checksum_ok);

	char no_star[] = "$GSSET,2";
	EXPECT_FALSE(parse(no_star, s));

	char no_dollar[] = "GSSET,2*00";
	EXPECT_FALSE(parse(no_dollar, s));

	char bad_hex[] = "$GSSET,2*ZZ";
	EXPECT_FALSE(parse(bad_hex, s));
}

TEST(ApntNmea, EmptyFieldsAreUnknown)
{
	Parsed p("GSPOS,3,,,,,,");
	ASSERT_TRUE(p.framed);
	EXPECT_EQ(p.s.field(3), nullptr);
	EXPECT_TRUE(isnan(get_double(p.s, 5)));
	EXPECT_EQ(get_int(p.s, 21), -1);
	EXPECT_EQ(get_int(p.s, 99, 7), 7);
}

TEST(ApntNmea, GpsTimeKeepsFractionExactly)
{
	// 1.4e9 s with 8 fractional digits needs more than a double holds
	EXPECT_EQ(parse_gps_time_us("1450000123.12345678"), 1450000123123456ULL);
	EXPECT_EQ(parse_gps_time_us("1450000123"), 1450000123000000ULL);
	EXPECT_EQ(parse_gps_time_us("12.5"), 12500000ULL);
	EXPECT_EQ(parse_gps_time_us(""), 0u);
	EXPECT_EQ(parse_gps_time_us(nullptr), 0u);
	EXPECT_EQ(parse_gps_time_us("abc"), 0u);
	EXPECT_EQ(parse_gps_time_us("12.5x"), 0u);
}

TEST(ApntNmea, DecodePosition)
{
	// 22 fields before the mode, position mode 3 (dynamic), PFOM 4
	Parsed p("GSPOS,3,1450000123.12345678,0,37.5,-122.25,120.5,1.0,2.0,3.0,"
		 "4.0,9.0,16.0,0.5,0.25,0.125,1.5,,2.5,,4,3,,,");
	ASSERT_TRUE(p.framed && p.s.checksum_ok);
	Pos o;
	ASSERT_TRUE(decode(p.s, o));
	EXPECT_EQ(o.gps_time_us, 1450000123123456ULL);
	EXPECT_DOUBLE_EQ(o.lat, 37.5);
	EXPECT_DOUBLE_EQ(o.lon, -122.25);
	EXPECT_DOUBLE_EQ(o.alt, 120.5);
	EXPECT_DOUBLE_EQ(o.ecef[2], 3.0);
	EXPECT_FLOAT_EQ(o.burst_age, 1.5f);
	EXPECT_FLOAT_EQ(o.kf_age, 2.5f);
	EXPECT_EQ(o.pfom, 4);
	EXPECT_EQ(o.mode, 3);

	// Row-major ENU: variances on the diagonal, E-N, E-U, N-U off diagonal
	EXPECT_FLOAT_EQ(o.cov[0], 4.f);
	EXPECT_FLOAT_EQ(o.cov[4], 9.f);
	EXPECT_FLOAT_EQ(o.cov[8], 16.f);
	EXPECT_FLOAT_EQ(o.cov[1], 0.5f);
	EXPECT_FLOAT_EQ(o.cov[3], 0.5f);
	EXPECT_FLOAT_EQ(o.cov[2], 0.25f);
	EXPECT_FLOAT_EQ(o.cov[6], 0.25f);
	EXPECT_FLOAT_EQ(o.cov[5], 0.125f);
	EXPECT_FLOAT_EQ(o.cov[7], 0.125f);
}

TEST(ApntNmea, DecodeVelocity)
{
	Parsed p("GSVEL,2,1450000123.5,1.0,2.0,-3.0,0.1,0.2,0.3,0.01,0.02,0.03,1.0,2.0,3.0,3");
	ASSERT_TRUE(p.framed && p.s.checksum_ok);
	Vel o;
	ASSERT_TRUE(decode(p.s, o));
	EXPECT_EQ(o.gps_time_us, 1450000123500000ULL);
	EXPECT_FLOAT_EQ(o.vel_enu[0], 1.f);
	EXPECT_FLOAT_EQ(o.vel_enu[1], 2.f);
	EXPECT_FLOAT_EQ(o.vel_enu[2], -3.f);
	EXPECT_FLOAT_EQ(o.cov[4], 0.2f);
	EXPECT_FLOAT_EQ(o.cov[5], 0.03f);
	EXPECT_FLOAT_EQ(o.sensor_age, 2.f);
	EXPECT_EQ(o.state, 3);
}

TEST(ApntNmea, DecodePps)
{
	// Empty UTC offset means invalid, empty TFOM is unknown
	Parsed p("GSPPS,6,1450000123,,0,1,1,350.5,,0,0,1,4.0,-12.5,420.0,,,,,");
	ASSERT_TRUE(p.framed && p.s.checksum_ok);
	Pps o;
	ASSERT_TRUE(decode(p.s, o));
	EXPECT_EQ(o.gps_time_s, 1450000123);
	EXPECT_EQ(o.utc_offset_s, -1);
	EXPECT_EQ(o.status, 1);
	EXPECT_EQ(o.source, 1);
	EXPECT_FLOAT_EQ(o.accuracy_ns, 350.5f);
	EXPECT_EQ(o.tfom, -1);
	EXPECT_FALSE(o.hard_correction_coming);
	EXPECT_EQ(o.iridium_status, 1);
	EXPECT_FLOAT_EQ(o.iridium_diff_ns, -12.5f);
	EXPECT_FLOAT_EQ(o.iridium_accuracy_ns, 420.f);
}

TEST(ApntNmea, DecodeSystemHexAndVersion)
{
	Parsed p("GSSYS,3,1000,990,0x40,0x033,0x033,1450000000,1502840827,,,C3.7.0.40086,,,,");
	ASSERT_TRUE(p.framed && p.s.checksum_ok);
	Sys o;
	ASSERT_TRUE(decode(p.s, o));
	EXPECT_EQ(o.uptime_s, 1000u);
	EXPECT_EQ(o.error, 0x40u);
	EXPECT_EQ(o.sub_configured, 0x33u);
	EXPECT_EQ(o.sub_end_gps_s, 1502840827);
	EXPECT_STREQ(o.version, "C3.7.0.40086");
}

TEST(ApntNmea, DecodeStatsAndSvi)
{
	Parsed a("GSSTA,3,,,,,100,6,35.5,40.0,30.0,,,");
	ASSERT_TRUE(a.framed);
	Sta sta;
	ASSERT_TRUE(decode(a.s, sta));
	EXPECT_EQ(sta.bursts_total, 100);
	EXPECT_EQ(sta.bursts_last_minute, 6);
	EXPECT_FLOAT_EQ(sta.cn0_max, 40.f);

	// Negative SV id (logged only) and empty pseudorange are legal
	Parsed v("GSSVI,2,-5,1450000123.5,12.25,77,,,41.5,-1200.5,1.0,2.0,3.0,4.0,5.0,6.0,2");
	ASSERT_TRUE(v.framed);
	Svi svi;
	ASSERT_TRUE(decode(v.s, svi));
	EXPECT_EQ(svi.sv_id, -5);
	EXPECT_EQ(svi.burst, 77);
	EXPECT_TRUE(isnan(svi.pseudorange));
	EXPECT_FLOAT_EQ(svi.doppler, -1200.5f);
	EXPECT_DOUBLE_EQ(svi.sv_pos[2], 3.0);
	EXPECT_FLOAT_EQ(svi.sv_vel[0], 4.f);
	EXPECT_EQ(svi.state, 2);
}

TEST(ApntNmea, DecodeSkyviewEntryWithMissingAngles)
{
	Parsed p("GSSSV,3,2,1,4,17,,,30.5,38.0");
	ASSERT_TRUE(p.framed);
	Ssv o;
	ASSERT_TRUE(decode(p.s, o));
	EXPECT_EQ(o.group_total, 2);
	EXPECT_EQ(o.group_index, 1);
	EXPECT_EQ(o.sv_count, 4);
	EXPECT_EQ(o.sv_id, 17);
	EXPECT_EQ(o.elevation, -1);
	EXPECT_EQ(o.azimuth, -1);
	EXPECT_FLOAT_EQ(o.cn0_max, 38.f);
}

TEST(ApntNmea, DecodeResponseAndParameterReport)
{
	Parsed r("GSRSP,3,42,0,-7");
	ASSERT_TRUE(r.framed);
	Rsp rsp;
	ASSERT_TRUE(decode(r.s, rsp));
	EXPECT_EQ(rsp.count, 42);
	EXPECT_FALSE(rsp.ack);
	EXPECT_EQ(rsp.error, -7);

	Parsed p("GSPAR,2,102,2,42,4,30.0,300.0,5.0,60.0");
	ASSERT_TRUE(p.framed);
	Par par;
	ASSERT_TRUE(decode(p.s, par));
	EXPECT_EQ(par.count, 42);
	EXPECT_EQ(par.param_id, 102);
	EXPECT_EQ(par.num_args, 4u);
	EXPECT_STREQ(par.args, "30.0,300.0,5.0,60.0");
}

TEST(ApntNmea, ParameterReportWithTooManyArgsIsTruncatedSafely)
{
	char body[MAX_LINE_LENGTH];
	int n = snprintf(body, sizeof(body), "GSPAR,2,5,2,1,%d", 12);

	for (int i = 0; i < 12 && n < (int)sizeof(body) - 20; i++) {
		n += snprintf(body + n, sizeof(body) - n, ",abcdefghijkl");
	}

	Parsed p(body);
	ASSERT_TRUE(p.framed);
	Par par;
	ASSERT_TRUE(decode(p.s, par));
	EXPECT_LT(strlen(par.args), sizeof(par.args));
}

TEST(ApntNmea, WrongHeaderIsNotDecoded)
{
	Parsed p("GSVEL,2,1");
	Pos o;
	EXPECT_FALSE(decode(p.s, o));
}
