/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/gnss/RelPosNED.hpp>

#include <lib/drivers/device/Device.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_gnss_relative.h>

namespace uavcannode
{

class RelPosHeadingPub :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::gnss::RelPosNED>
{
public:
	RelPosHeadingPub(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::gnss::RelPosNED::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(sensor_gnss_relative)),
		uavcan::Publisher<uavcan::equipment::gnss::RelPosNED>(node)
	{
		this->setPriority(uavcan::TransferPriority::NumericallyMax);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::gnss::RelPosNED::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		using uavcan::equipment::gnss::RelPosNED;

		// sensor_gnss_relative -> uavcan::equipment::gnss::RelPosNED
		sensor_gnss_relative_s sensor_gnss_relative;

		if (uORB::SubscriptionCallbackWorkItem::update(&sensor_gnss_relative)) {
			RelPosNED rel_pos{};

			// TODO: FIX (timestamp_sample and UAVCAN timestamp)
			rel_pos.timestamp = uavcan::UtcTime::fromUSec(getNode().getMonotonicTime().toUSec());
			rel_pos.reference_station_id = sensor_gnss_relative.reference_station_id;

			for (unsigned i = 0; i < 3; i++) {
				rel_pos.position[i] = sensor_gnss_relative.position[i];
				rel_pos.position_accuracy[i] = sensor_gnss_relative.position_accuracy[i];
			}

			rel_pos.heading = sensor_gnss_relative.heading;
			rel_pos.heading_accuracy = sensor_gnss_relative.heading_accuracy;
			rel_pos.position_length = sensor_gnss_relative.position_length;
			rel_pos.accuracy_length = sensor_gnss_relative.accuracy_length;
			rel_pos.gnss_fix_ok = sensor_gnss_relative.gnss_fix_ok;
			rel_pos.differential_solution = sensor_gnss_relative.differential_solution;
			rel_pos.relative_position_valid = sensor_gnss_relative.relative_position_valid;
			rel_pos.carrier_solution_floating = sensor_gnss_relative.carrier_solution_floating;
			rel_pos.carrier_solution_fixed = sensor_gnss_relative.carrier_solution_fixed;
			rel_pos.moving_base_mode = sensor_gnss_relative.moving_base_mode;
			rel_pos.reference_position_miss = sensor_gnss_relative.reference_position_miss;
			rel_pos.reference_observations_miss = sensor_gnss_relative.reference_observations_miss;
			rel_pos.heading_valid = sensor_gnss_relative.heading_valid;
			rel_pos.relative_position_normalized = sensor_gnss_relative.relative_position_normalized;

			uavcan::Publisher<RelPosNED>::broadcast(rel_pos);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
