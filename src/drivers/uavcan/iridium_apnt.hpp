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

#include <uavcan/uavcan.hpp>
#include <iridium/apnt/Position.hpp>
#include <iridium/apnt/Velocity.hpp>
#include <iridium/apnt/Pps.hpp>
#include <iridium/apnt/System.hpp>
#include <iridium/apnt/Stats.hpp>
#include <iridium/apnt/Svi.hpp>
#include <iridium/apnt/Skyview.hpp>
#include <iridium/apnt/Sentence.hpp>
#include <iridium/apnt/Command.hpp>
#include <iridium/apnt/Response.hpp>

#include <uORB/Publication.hpp>
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

/**
 * Receives the iridium.apnt DSDL messages sent by a CAN node running the iridium_apnt driver and
 * republishes them as iridium_apnt_* uORB topics. The device_id carries the source node ID, so
 * several nodes stay distinguishable. iridium_apnt_command is forwarded to the nodes as
 * iridium.apnt.Command (addressed to all nodes).
 *
 * The position and velocity solution itself arrives as uavcan.equipment.gnss.Fix2 and is handled
 * by the GNSS sensor bridge.
 */
class UavcanIridiumApnt
{
public:
	explicit UavcanIridiumApnt(uavcan::INode &node);

	int init();

private:
	template<typename Dsdl>
	using Binder = uavcan::MethodBinder<UavcanIridiumApnt *,
	      void (UavcanIridiumApnt::*)(const uavcan::ReceivedDataStructure<Dsdl> &)>;

	using TimerCbBinder = uavcan::MethodBinder<UavcanIridiumApnt *, void (UavcanIridiumApnt::*)(const uavcan::TimerEvent &)>;

	template<typename Dsdl>
	int start(uavcan::Subscriber<Dsdl, Binder<Dsdl>> &sub);

	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::Position> &msg);
	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::Velocity> &msg);
	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::Pps> &msg);
	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::System> &msg);
	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::Stats> &msg);
	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::Svi> &msg);
	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::Skyview> &msg);
	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::Sentence> &msg);
	void handle(const uavcan::ReceivedDataStructure<iridium::apnt::Response> &msg);

	void forward_commands(const uavcan::TimerEvent &);

	uavcan::Subscriber<iridium::apnt::Position, Binder<iridium::apnt::Position>> _sub_position;
	uavcan::Subscriber<iridium::apnt::Velocity, Binder<iridium::apnt::Velocity>> _sub_velocity;
	uavcan::Subscriber<iridium::apnt::Pps, Binder<iridium::apnt::Pps>> _sub_pps;
	uavcan::Subscriber<iridium::apnt::System, Binder<iridium::apnt::System>> _sub_system;
	uavcan::Subscriber<iridium::apnt::Stats, Binder<iridium::apnt::Stats>> _sub_stats;
	uavcan::Subscriber<iridium::apnt::Svi, Binder<iridium::apnt::Svi>> _sub_svi;
	uavcan::Subscriber<iridium::apnt::Skyview, Binder<iridium::apnt::Skyview>> _sub_skyview;
	uavcan::Subscriber<iridium::apnt::Sentence, Binder<iridium::apnt::Sentence>> _sub_sentence;
	uavcan::Subscriber<iridium::apnt::Response, Binder<iridium::apnt::Response>> _sub_response;

	uavcan::Publisher<iridium::apnt::Command> _pub_command;
	uavcan::TimerEventForwarder<TimerCbBinder> _command_timer;

	uORB::Subscription _command_sub{ORB_ID(iridium_apnt_command)};

	uORB::Publication<iridium_apnt_position_s> _position_pub{ORB_ID(iridium_apnt_position)};
	uORB::Publication<iridium_apnt_velocity_s> _velocity_pub{ORB_ID(iridium_apnt_velocity)};
	uORB::Publication<iridium_apnt_pps_s> _pps_pub{ORB_ID(iridium_apnt_pps)};
	uORB::Publication<iridium_apnt_system_s> _system_pub{ORB_ID(iridium_apnt_system)};
	uORB::Publication<iridium_apnt_stats_s> _stats_pub{ORB_ID(iridium_apnt_stats)};
	uORB::Publication<iridium_apnt_svi_s> _svi_pub{ORB_ID(iridium_apnt_svi)};
	uORB::Publication<iridium_apnt_skyview_s> _skyview_pub{ORB_ID(iridium_apnt_skyview)};
	uORB::Publication<iridium_apnt_sentence_s> _sentence_pub{ORB_ID(iridium_apnt_sentence)};
	uORB::Publication<iridium_apnt_response_s> _response_pub{ORB_ID(iridium_apnt_response)};
};
