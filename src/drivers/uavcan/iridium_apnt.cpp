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

#include "iridium_apnt.hpp"

#include <cmath>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>

namespace
{

uint32_t make_device_id(uint8_t node_id)
{
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType_UAVCAN;
	id.devid_s.bus = 0;
	id.devid_s.address = node_id;
	id.devid_s.devtype = DRV_GPS_DEVTYPE_IRIDIUM_APNT;
	return id.devid;
}

// Inverse of the upper triangle packing used by uavcannode: EE, EN, EU, NN, NU, UU
template<typename Array>
void expand_covariance(const Array &tri, float out[9])
{
	out[0] = tri[0]; out[1] = tri[1]; out[2] = tri[2];
	out[3] = tri[1]; out[4] = tri[3]; out[5] = tri[4];
	out[6] = tri[2]; out[7] = tri[4]; out[8] = tri[5];
}

template<typename Array>
void array_to_string(const Array &in, uint8_t *out, size_t out_size)
{
	size_t n = 0;

	for (; n < in.size() && n + 1 < out_size; n++) {
		out[n] = in[n];
	}

	out[n] = 0;
}

// WGS84 geodetic to ECEF
void geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m, double ecef[3])
{
	constexpr double a = 6378137.0;
	constexpr double e2 = 6.69437999014e-3;
	const double lat = lat_deg * M_PI / 180.0;
	const double lon = lon_deg * M_PI / 180.0;
	const double n = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
	ecef[0] = (n + alt_m) * std::cos(lat) * std::cos(lon);
	ecef[1] = (n + alt_m) * std::cos(lat) * std::sin(lon);
	ecef[2] = (n * (1.0 - e2) + alt_m) * std::sin(lat);
}

} // namespace

UavcanIridiumApnt::UavcanIridiumApnt(uavcan::INode &node) :
	_sub_position(node),
	_sub_velocity(node),
	_sub_pps(node),
	_sub_system(node),
	_sub_stats(node),
	_sub_svi(node),
	_sub_skyview(node),
	_sub_sentence(node),
	_sub_response(node),
	_pub_command(node),
	_command_timer(node)
{
}

template<typename Dsdl>
int UavcanIridiumApnt::start(uavcan::Subscriber<Dsdl, Binder<Dsdl>> &sub)
{
	const int res = sub.start(Binder<Dsdl>(this, &UavcanIridiumApnt::handle));

	if (res < 0) {
		PX4_WARN("%s subscription failed %d", Dsdl::getDataTypeFullName(), res);
	}

	return res;
}

int UavcanIridiumApnt::init()
{
	int res = 0;

	if ((res = start(_sub_position)) < 0) { return res; }

	if ((res = start(_sub_velocity)) < 0) { return res; }

	if ((res = start(_sub_pps)) < 0) { return res; }

	if ((res = start(_sub_system)) < 0) { return res; }

	if ((res = start(_sub_stats)) < 0) { return res; }

	if ((res = start(_sub_svi)) < 0) { return res; }

	if ((res = start(_sub_skyview)) < 0) { return res; }

	if ((res = start(_sub_sentence)) < 0) { return res; }

	if ((res = start(_sub_response)) < 0) { return res; }

	_pub_command.setPriority(uavcan::TransferPriority::MiddleLower);

	_command_timer.setCallback(TimerCbBinder(this, &UavcanIridiumApnt::forward_commands));
	_command_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(50));

	return 0;
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::Position> &msg)
{
	iridium_apnt_position_s out{};
	out.timestamp = hrt_absolute_time();
	out.device_id = make_device_id(msg.getSrcNodeID().get());
	out.gps_time_us = msg.gps_time_us;
	out.latitude = msg.latitude_deg_1e8 * 1e-8;
	out.longitude = msg.longitude_deg_1e8 * 1e-8;
	out.altitude_ellipsoid = msg.height_ellipsoid_mm * 1e-3;
	geodetic_to_ecef(out.latitude, out.longitude, out.altitude_ellipsoid, out.ecef);
	expand_covariance(msg.covariance, out.covariance);
	out.burst_age = msg.burst_age;
	out.kf_age = msg.kf_age;
	out.pfom = msg.pfom;
	out.mode = msg.mode;
	_position_pub.publish(out);
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::Velocity> &msg)
{
	iridium_apnt_velocity_s out{};
	out.timestamp = hrt_absolute_time();
	out.device_id = make_device_id(msg.getSrcNodeID().get());
	out.gps_time_us = msg.gps_time_us;

	for (int i = 0; i < 3; i++) {
		out.velocity_enu[i] = msg.velocity_enu[i];
	}

	expand_covariance(msg.covariance, out.covariance);
	out.burst_age = msg.burst_age;
	out.sensor_age = msg.sensor_age;
	out.kf_age = msg.kf_age;
	out.state = msg.state;
	_velocity_pub.publish(out);
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::Pps> &msg)
{
	iridium_apnt_pps_s out{};
	out.timestamp = hrt_absolute_time();
	out.device_id = make_device_id(msg.getSrcNodeID().get());
	out.gps_time_s = msg.gps_time_s;
	out.utc_offset_s = msg.utc_offset_s;
	out.leap = msg.leap;
	out.pps_status = msg.pps_status;
	out.source = msg.source;
	out.accuracy_ns = msg.accuracy_ns;
	out.tfom = msg.tfom;
	out.hard_correction_coming = msg.hard_correction_coming;
	out.hard_correction_applied = msg.hard_correction_applied;
	out.iridium_pps_status = msg.iridium_pps_status;
	out.burst_age = msg.burst_age;
	out.iridium_diff_ns = msg.iridium_diff_ns;
	out.iridium_accuracy_ns = msg.iridium_accuracy_ns;
	_pps_pub.publish(out);
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::System> &msg)
{
	iridium_apnt_system_s out{};
	out.timestamp = hrt_absolute_time();
	out.device_id = make_device_id(msg.getSrcNodeID().get());
	out.uptime_s = msg.uptime_s;
	out.app_uptime_s = msg.app_uptime_s;
	out.error = msg.error;
	out.subscription_configured = msg.subscription_configured;
	out.subscription_enabled = msg.subscription_enabled;
	out.subscription_start_gps_s = msg.subscription_start_gps_s;
	out.subscription_end_gps_s = msg.subscription_end_gps_s;
	out.extended_status = msg.extended_status;
	array_to_string(msg.version, out.version, sizeof(out.version));
	_system_pub.publish(out);
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::Stats> &msg)
{
	iridium_apnt_stats_s out{};
	out.timestamp = hrt_absolute_time();
	out.device_id = make_device_id(msg.getSrcNodeID().get());
	out.bursts_total = msg.bursts_total;
	out.bursts_last_minute = msg.bursts_last_minute;
	out.cn0_avg = msg.cn0_avg;
	out.cn0_max = msg.cn0_max;
	out.cn0_min = msg.cn0_min;
	_stats_pub.publish(out);
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::Svi> &msg)
{
	iridium_apnt_svi_s out{};
	out.timestamp = hrt_absolute_time();
	out.device_id = make_device_id(msg.getSrcNodeID().get());
	out.sv_id = msg.sv_id;
	out.time_of_validity_us = msg.time_of_validity_us;
	out.time_of_arrival_s = msg.time_of_arrival_s;
	out.burst_counter = msg.burst_counter;
	out.pseudorange = msg.pseudorange;
	out.pseudorange_rate = msg.pseudorange_rate;
	out.cn0 = msg.cn0;
	out.doppler = msg.doppler;

	for (int i = 0; i < 3; i++) {
		out.sv_position_ecef[i] = msg.sv_position_ecef[i];
		out.sv_velocity_ecef[i] = msg.sv_velocity_ecef[i];
	}

	out.state = msg.state;
	_svi_pub.publish(out);
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::Skyview> &msg)
{
	iridium_apnt_skyview_s out{};
	out.timestamp = hrt_absolute_time();
	out.device_id = make_device_id(msg.getSrcNodeID().get());
	out.satellites_in_view = msg.satellites_in_view;

	// The parallel arrays are always pushed together; guard against a peer that disagrees
	size_t n = msg.sv_id.size();

	n = n < msg.elevation_deg.size() ? n : msg.elevation_deg.size();
	n = n < msg.azimuth_deg.size() ? n : msg.azimuth_deg.size();
	n = n < msg.cn0_min.size() ? n : msg.cn0_min.size();
	n = n < msg.cn0_max.size() ? n : msg.cn0_max.size();
	n = n < iridium_apnt_skyview_s::MAX_SATELLITES ? n : iridium_apnt_skyview_s::MAX_SATELLITES;

	for (size_t i = 0; i < n; i++) {
		out.sv_id[i] = msg.sv_id[i];
		out.elevation[i] = msg.elevation_deg[i];
		out.azimuth[i] = msg.azimuth_deg[i];
		out.cn0_min[i] = msg.cn0_min[i];
		out.cn0_max[i] = msg.cn0_max[i];
	}

	out.satellite_count = n;
	_skyview_pub.publish(out);
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::Sentence> &msg)
{
	iridium_apnt_sentence_s out{};
	out.timestamp = hrt_absolute_time();
	out.device_id = make_device_id(msg.getSrcNodeID().get());
	out.checksum_ok = msg.checksum_ok;

	const size_t n = msg.data.size() < sizeof(out.data) ? msg.data.size() : sizeof(out.data);

	for (size_t i = 0; i < n; i++) {
		out.data[i] = msg.data[i];
	}

	out.len = n;
	_sentence_pub.publish(out);
}

void UavcanIridiumApnt::handle(const uavcan::ReceivedDataStructure<iridium::apnt::Response> &msg)
{
	iridium_apnt_response_s out{};
	out.timestamp = hrt_absolute_time();
	out.request_id = msg.request_id;
	out.ack = msg.ack;
	out.timeout = msg.timeout;
	out.error_code = msg.error_code;
	out.param_id = msg.param_id;
	array_to_string(msg.args, out.args, sizeof(out.args));
	_response_pub.publish(out);
}

void UavcanIridiumApnt::forward_commands(const uavcan::TimerEvent &)
{
	iridium_apnt_command_s cmd;

	while (_command_sub.update(&cmd)) {
		iridium::apnt::Command out{};
		out.target_node_id = 0;
		out.request_id = cmd.request_id;
		out.type = cmd.type;
		out.param_id = cmd.param_id;
		out.nvm = cmd.nvm;

		for (size_t i = 0; i < sizeof(cmd.args) && cmd.args[i] != 0 && out.args.size() < out.args.capacity(); i++) {
			out.args.push_back(cmd.args[i]);
		}

		_pub_command.broadcast(out);
	}
}
