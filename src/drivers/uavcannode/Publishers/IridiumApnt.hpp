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

#include <cmath>
#include <cstring>

#include "UavcanPublisherBase.hpp"

#include <iridium/apnt/Position.hpp>
#include <iridium/apnt/Velocity.hpp>
#include <iridium/apnt/Pps.hpp>
#include <iridium/apnt/System.hpp>
#include <iridium/apnt/Stats.hpp>
#include <iridium/apnt/Svi.hpp>
#include <iridium/apnt/Skyview.hpp>
#include <iridium/apnt/Sentence.hpp>
#include <iridium/apnt/Response.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/iridium_apnt_position.h>
#include <uORB/topics/iridium_apnt_velocity.h>
#include <uORB/topics/iridium_apnt_pps.h>
#include <uORB/topics/iridium_apnt_system.h>
#include <uORB/topics/iridium_apnt_stats.h>
#include <uORB/topics/iridium_apnt_svi.h>
#include <uORB/topics/iridium_apnt_skyview.h>
#include <uORB/topics/iridium_apnt_sentence.h>
#include <uORB/topics/iridium_apnt_response.h>

namespace uavcannode
{

namespace iridium_apnt_convert
{

// DSDL carries the upper triangle of a symmetric row-major 3x3: EE, EN, EU, NN, NU, UU
template<typename Array>
inline void upper_triangle(const float cov[9], Array &out)
{
	static constexpr int idx[6] {0, 1, 2, 4, 5, 8};

	for (int i = 0; i < 6; i++) {
		out[i] = cov[idx[i]];
	}
}

// Copy a NUL terminated uORB byte string into a dynamic DSDL byte array
template<typename Array>
inline void string_to_array(const uint8_t *src, size_t max_len, Array &out)
{
	for (size_t i = 0; i < max_len && src[i] != 0 && out.size() < out.capacity(); i++) {
		out.push_back(src[i]);
	}
}

inline void convert(const iridium_apnt_position_s &in, iridium::apnt::Position &out)
{
	out.gps_time_us = in.gps_time_us;
	out.latitude_deg_1e8 = static_cast<int64_t>(in.latitude * 1e8);
	out.longitude_deg_1e8 = static_cast<int64_t>(in.longitude * 1e8);
	out.height_ellipsoid_mm = static_cast<int32_t>(in.altitude_ellipsoid * 1e3);
	upper_triangle(in.covariance, out.covariance);
	out.burst_age = in.burst_age;
	out.kf_age = in.kf_age;
	out.pfom = in.pfom;
	out.mode = in.mode;
}

inline void convert(const iridium_apnt_velocity_s &in, iridium::apnt::Velocity &out)
{
	out.gps_time_us = in.gps_time_us;

	for (int i = 0; i < 3; i++) {
		out.velocity_enu[i] = in.velocity_enu[i];
	}

	upper_triangle(in.covariance, out.covariance);
	out.burst_age = in.burst_age;
	out.sensor_age = in.sensor_age;
	out.kf_age = in.kf_age;
	out.state = in.state;
}

inline void convert(const iridium_apnt_pps_s &in, iridium::apnt::Pps &out)
{
	out.gps_time_s = in.gps_time_s;
	out.utc_offset_s = in.utc_offset_s;
	out.leap = in.leap;
	out.pps_status = in.pps_status;
	out.source = in.source;
	out.accuracy_ns = in.accuracy_ns;
	out.tfom = in.tfom;
	out.hard_correction_coming = in.hard_correction_coming;
	out.hard_correction_applied = in.hard_correction_applied;
	out.iridium_pps_status = in.iridium_pps_status;
	out.burst_age = in.burst_age;
	out.iridium_diff_ns = in.iridium_diff_ns;
	out.iridium_accuracy_ns = in.iridium_accuracy_ns;
}

inline void convert(const iridium_apnt_system_s &in, iridium::apnt::System &out)
{
	out.uptime_s = static_cast<uint32_t>(in.uptime_s);
	out.app_uptime_s = static_cast<uint32_t>(in.app_uptime_s);
	out.error = in.error;
	out.subscription_configured = in.subscription_configured;
	out.subscription_enabled = in.subscription_enabled;
	out.subscription_start_gps_s = in.subscription_start_gps_s;
	out.subscription_end_gps_s = in.subscription_end_gps_s;
	out.extended_status = in.extended_status;
	string_to_array(in.version, sizeof(in.version), out.version);
}

inline void convert(const iridium_apnt_stats_s &in, iridium::apnt::Stats &out)
{
	out.bursts_total = in.bursts_total;
	out.bursts_last_minute = in.bursts_last_minute;
	out.cn0_avg = in.cn0_avg;
	out.cn0_max = in.cn0_max;
	out.cn0_min = in.cn0_min;
}

inline void convert(const iridium_apnt_svi_s &in, iridium::apnt::Svi &out)
{
	out.sv_id = in.sv_id;
	out.time_of_validity_us = in.time_of_validity_us;
	out.time_of_arrival_s = in.time_of_arrival_s;
	out.burst_counter = in.burst_counter;
	out.pseudorange = in.pseudorange;
	out.pseudorange_rate = in.pseudorange_rate;
	out.cn0 = in.cn0;
	out.doppler = in.doppler;

	for (int i = 0; i < 3; i++) {
		out.sv_position_ecef[i] = in.sv_position_ecef[i];
		out.sv_velocity_ecef[i] = in.sv_velocity_ecef[i];
	}

	out.state = in.state;
}

inline void convert(const iridium_apnt_skyview_s &in, iridium::apnt::Skyview &out)
{
	out.satellites_in_view = in.satellites_in_view;

	for (uint8_t i = 0; i < in.satellite_count && i < iridium_apnt_skyview_s::MAX_SATELLITES; i++) {
		out.sv_id.push_back(in.sv_id[i]);
		out.elevation_deg.push_back(in.elevation[i]);
		out.azimuth_deg.push_back(in.azimuth[i]);
		out.cn0_min.push_back(in.cn0_min[i]);
		out.cn0_max.push_back(in.cn0_max[i]);
	}
}

inline void convert(const iridium_apnt_sentence_s &in, iridium::apnt::Sentence &out)
{
	out.checksum_ok = in.checksum_ok;

	for (size_t i = 0; i < in.len && i < sizeof(in.data) && out.data.size() < out.data.capacity(); i++) {
		out.data.push_back(in.data[i]);
	}
}

inline void convert(const iridium_apnt_response_s &in, iridium::apnt::Response &out)
{
	out.request_id = in.request_id;
	out.ack = in.ack;
	out.timeout = in.timeout;
	out.error_code = in.error_code;
	out.param_id = in.param_id;
	string_to_array(in.args, sizeof(in.args), out.args);
}

} // namespace iridium_apnt_convert

/**
 * Publishes one iridium_apnt_* uORB topic as its iridium.apnt DSDL message. Topics that are
 * queued (per-burst data, raw sentences, command responses) are drained on every wake-up.
 */
template<typename Dsdl, typename OrbStruct>
class IridiumApntPublisher :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<Dsdl>
{
public:
	IridiumApntPublisher(px4::WorkItem *work_item, uavcan::INode &node, const orb_metadata *meta,
			     uavcan::TransferPriority priority = uavcan::TransferPriority::MiddleLower) :
		UavcanPublisherBase(Dsdl::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, meta),
		uavcan::Publisher<Dsdl>(node)
	{
		this->setPriority(priority);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       Dsdl::getDataTypeFullName(),
			       Dsdl::DefaultDataTypeID);
		}
	}

	void BroadcastAnyUpdates() override
	{
		OrbStruct msg;

		// A burst of queued messages must not starve the work queue
		for (int i = 0; i < kMaxPerRun && uORB::SubscriptionCallbackWorkItem::update(&msg); i++) {
			Dsdl dsdl{};
			iridium_apnt_convert::convert(msg, dsdl);
			uavcan::Publisher<Dsdl>::broadcast(dsdl);
		}

		// ensure callback is registered
		uORB::SubscriptionCallbackWorkItem::registerCallback();
	}

private:
	static constexpr int kMaxPerRun{8};
};

using IridiumApntPosition = IridiumApntPublisher<iridium::apnt::Position, iridium_apnt_position_s>;
using IridiumApntVelocity = IridiumApntPublisher<iridium::apnt::Velocity, iridium_apnt_velocity_s>;
using IridiumApntPps = IridiumApntPublisher<iridium::apnt::Pps, iridium_apnt_pps_s>;
using IridiumApntSystem = IridiumApntPublisher<iridium::apnt::System, iridium_apnt_system_s>;
using IridiumApntStats = IridiumApntPublisher<iridium::apnt::Stats, iridium_apnt_stats_s>;
using IridiumApntSvi = IridiumApntPublisher<iridium::apnt::Svi, iridium_apnt_svi_s>;
using IridiumApntSkyview = IridiumApntPublisher<iridium::apnt::Skyview, iridium_apnt_skyview_s>;
using IridiumApntSentence = IridiumApntPublisher<iridium::apnt::Sentence, iridium_apnt_sentence_s>;
using IridiumApntResponse = IridiumApntPublisher<iridium::apnt::Response, iridium_apnt_response_s>;

} // namespace uavcannode
