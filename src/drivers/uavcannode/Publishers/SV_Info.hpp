/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/gnss/SV_Info.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/satellite_info.h>

namespace uavcannode
{

class SVInfoPub :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::gnss::SV_Info>
{
public:
	SVInfoPub(px4::WorkItem *work_item, uavcan::INode &node, uint8_t instance) :
		UavcanPublisherBase(uavcan::equipment::gnss::SV_Info::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(satellite_info), instance),
		uavcan::Publisher<uavcan::equipment::gnss::SV_Info>(node)
	{
		this->setPriority(uavcan::TransferPriority::MiddleLower);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s[%u] -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uORB::SubscriptionCallbackWorkItem::get_instance(),
			       uavcan::equipment::gnss::SV_Info::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		satellite_info_s satellite_info{};

		if (uORB::SubscriptionCallbackWorkItem::update(&satellite_info)) {
			uavcan::equipment::gnss::SV_Info sv_info{};

			sv_info.timestamp = uavcan::UtcTime::fromUSec(getNode().getMonotonicTime().toUSec());

			const size_t count = satellite_info.count < sv_info.svid.capacity() ?
					     satellite_info.count : sv_info.svid.capacity();

			for (size_t i = 0; i < count; i++) {
				sv_info.svid.push_back(satellite_info.svid[i]);
				sv_info.used.push_back(satellite_info.used[i]);
				sv_info.elevation.push_back(satellite_info.elevation[i]);
				sv_info.azimuth.push_back(satellite_info.azimuth[i]);
				sv_info.snr.push_back(satellite_info.snr[i]);
				sv_info.prn.push_back(satellite_info.prn[i]);
			}

			uavcan::Publisher<uavcan::equipment::gnss::SV_Info>::broadcast(sv_info);

			// Ensure callback is registered.
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
