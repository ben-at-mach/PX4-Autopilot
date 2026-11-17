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

#include "UavcanSubscriberBase.hpp"

#include <iridium/apnt/Command.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/iridium_apnt_command.h>

namespace uavcannode
{

class IridiumApntCommand;

typedef uavcan::MethodBinder<IridiumApntCommand *,
	void (IridiumApntCommand::*)(const uavcan::ReceivedDataStructure<iridium::apnt::Command>&)>
	IridiumApntCommandBinder;

/** Hands a iridium.apnt.Command addressed to this node to the iridium_apnt driver. */
class IridiumApntCommand :
	public UavcanSubscriberBase,
	private uavcan::Subscriber<iridium::apnt::Command, IridiumApntCommandBinder>
{
public:
	IridiumApntCommand(uavcan::INode &node) :
		UavcanSubscriberBase(iridium::apnt::Command::DefaultDataTypeID),
		uavcan::Subscriber<iridium::apnt::Command, IridiumApntCommandBinder>(node),
		_node(node)
	{}

	bool init() override
	{
		if (start(IridiumApntCommandBinder(this, &IridiumApntCommand::callback)) < 0) {
			PX4_ERR("iridium::apnt::Command subscription failed");
			return false;
		}

		return true;
	}

	void PrintInfo() const override
	{
		printf("\t%s:%d -> %s\n",
		       iridium::apnt::Command::getDataTypeFullName(),
		       iridium::apnt::Command::DefaultDataTypeID,
		       _command_pub.get_topic()->o_name);
	}

private:
	void callback(const uavcan::ReceivedDataStructure<iridium::apnt::Command> &msg)
	{
		if (msg.target_node_id != 0 && msg.target_node_id != _node.getNodeID().get()) {
			return;
		}

		iridium_apnt_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.request_id = msg.request_id;
		cmd.type = msg.type;
		cmd.param_id = msg.param_id;
		cmd.nvm = msg.nvm;

		// args is NUL terminated by the zero initialisation, and one byte shorter than the uORB field
		for (size_t i = 0; i < msg.args.size() && i < sizeof(cmd.args) - 1; i++) {
			cmd.args[i] = msg.args[i];
		}

		_command_pub.publish(cmd);
	}

	uavcan::INode &_node;
	uORB::Publication<iridium_apnt_command_s> _command_pub{ORB_ID(iridium_apnt_command)};
};
} // namespace uavcannode
