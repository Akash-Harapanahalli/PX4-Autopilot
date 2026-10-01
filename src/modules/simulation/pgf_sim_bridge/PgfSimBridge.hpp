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
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PgfSimBridge.hpp
 *
 * SIMULATION ONLY. Stands in for the companion computer so mc_pgf_control can
 * be flown in SITL without ROS 2, Gazebo or a Micro XRCE-DDS agent.
 *
 * On the real vehicle the value expansion arrives over uXRCE-DDS from the Jetson
 * and quad_px4_offboard supplies the offboard heartbeat. Here this module does
 * both over plain UDP against a Python process running the same acados MPC:
 *
 *   PX4 -> :14590   the 13-state estimate, every cycle
 *   :14591 -> PX4   the value expansion, published to the same uORB topics the
 *                   DDS client would have written
 *
 * It deliberately mirrors quad_px4_offboard's failsafe: the OffboardControlMode
 * heartbeat is withheld when the companion goes quiet, so PX4 runs its own
 * failsafe rather than flying on a stale expansion.
 *
 * What this does NOT exercise is the XRCE-DDS transport itself. That is what the
 * hardware echo test is for.
 */

#pragma once

#include <lib/pgf/PgfModel.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/pgf_value_expansion.h>
#include <uORB/topics/pgf_value_hessian.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <netinet/in.h>

class PgfSimBridge : public ModuleBase<PgfSimBridge>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	PgfSimBridge();
	~PgfSimBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

	// Wire format, native endianness: this only ever talks to localhost.
	// Packed so the layout is exactly the field sizes -- StatePacket would
	// otherwise be padded to 64 bytes for the uint64_t's alignment, and the
	// Python side has no way to see that.
	struct __attribute__((packed)) StatePacket {
		uint64_t timestamp_sample;
		float x[pgf::NX];
	};

	struct __attribute__((packed)) PayloadPacket {
		uint32_t seq;
		float dtau;
		float x0[pgf::NX];
		float x1[pgf::NX];
		float s0[pgf::NX];
		float s1[pgf::NX];
		float u0[pgf::NU];
		float u_nom0[pgf::NU];
		float u_nom1[pgf::NU];
		float s0_upper[91];
	};

	static_assert(sizeof(StatePacket) == 8 + 4 * pgf::NX, "StatePacket padded");
	static_assert(sizeof(PayloadPacket) == 8 + 4 * (4 * pgf::NX + 3 * pgf::NU + 91),
		      "PayloadPacket padded");

private:
	void Run() override;

	bool openSocket();
	void sendState();
	void receivePayloads();
	void publishHeartbeat();
	void requestOffboardAndArm();

	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<pgf_value_expansion_s> _expansion_pub{ORB_ID(pgf_value_expansion)};
	uORB::Publication<pgf_value_hessian_s> _hessian_pub{ORB_ID(pgf_value_hessian)};
	uORB::Publication<offboard_control_mode_s> _offboard_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	int _fd{-1};
	struct sockaddr_in _peer {};

	uint32_t _rx_count{0};
	uint32_t _tx_count{0};
	hrt_abstime _last_rx{0};
	hrt_abstime _started{0};
	bool _armed{false};
	bool _offboard{false};
	bool _engage_requested{false};

	int32_t _auto_engage{0};
	float _timeout_ms{200.f};

	perf_counter_t _loop_perf;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PGF_SIM_ENGAGE>) _param_engage,
		(ParamFloat<px4::params::PGF_SIM_TOUT>) _param_timeout
	)
};
