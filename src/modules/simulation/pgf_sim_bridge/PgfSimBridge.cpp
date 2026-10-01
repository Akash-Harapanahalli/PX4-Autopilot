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

#include "PgfSimBridge.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace pgf;
using namespace time_literals;

static constexpr uint16_t kListenPort = 14591;   ///< payloads in
static constexpr uint16_t kPeerPort = 14590;     ///< state out
static constexpr uint32_t kRateHz = 250;         ///< matches the SIH loop
/// Give the estimator time to settle before asking to arm.
static constexpr hrt_abstime kEngageDelay = 8_s;

PgfSimBridge::PgfSimBridge() :
	ModuleParams(nullptr),
	px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	updateParams();
	_auto_engage = _param_engage.get();
	_timeout_ms = _param_timeout.get();
}

PgfSimBridge::~PgfSimBridge()
{
	if (_fd >= 0) {
		close(_fd);
	}

	perf_free(_loop_perf);
}

bool PgfSimBridge::openSocket()
{
	_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_fd < 0) {
		PX4_ERR("socket: %s", strerror(errno));
		return false;
	}

	int flags = fcntl(_fd, F_GETFL, 0);
	fcntl(_fd, F_SETFL, flags | O_NONBLOCK);

	struct sockaddr_in addr {};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons(kListenPort);

	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind %u: %s", kListenPort, strerror(errno));
		close(_fd);
		_fd = -1;
		return false;
	}

	_peer.sin_family = AF_INET;
	_peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	_peer.sin_port = htons(kPeerPort);
	return true;
}

bool PgfSimBridge::init()
{
	if (!openSocket()) {
		return false;
	}

	_started = hrt_absolute_time();
	ScheduleOnInterval(1000000 / kRateHz);
	PX4_INFO("listening on %u, sending state to %u", kListenPort, kPeerPort);
	return true;
}

void PgfSimBridge::sendState()
{
	vehicle_local_position_s pos;
	vehicle_attitude_s att;
	vehicle_angular_velocity_s rates;

	if (!_local_position_sub.copy(&pos) || !_attitude_sub.copy(&att)
	    || !_angular_velocity_sub.copy(&rates)) {
		return;
	}

	if (!pos.xy_valid || !pos.z_valid) {
		return;
	}

	StatePacket pkt{};
	pkt.timestamp_sample = pos.timestamp_sample;
	pkt.x[IDX_P] = pos.x;
	pkt.x[IDX_P + 1] = pos.y;
	pkt.x[IDX_P + 2] = pos.z;
	pkt.x[IDX_V] = pos.vx;
	pkt.x[IDX_V + 1] = pos.vy;
	pkt.x[IDX_V + 2] = pos.vz;

	for (int i = 0; i < 4; i++) {
		pkt.x[IDX_Q + i] = att.q[i];
	}

	for (int i = 0; i < 3; i++) {
		pkt.x[IDX_W + i] = rates.xyz[i];
	}

	if (sendto(_fd, &pkt, sizeof(pkt), 0, (struct sockaddr *)&_peer, sizeof(_peer)) > 0) {
		_tx_count++;
	}
}

void PgfSimBridge::receivePayloads()
{
	PayloadPacket pkt;

	// Drain the socket: only the newest expansion matters.
	while (recv(_fd, &pkt, sizeof(pkt), 0) == (ssize_t)sizeof(pkt)) {
		const hrt_abstime now = hrt_absolute_time();

		pgf_value_expansion_s exp{};
		exp.timestamp_sample = now;
		exp.seq = pkt.seq;
		exp.dtau = pkt.dtau;

		for (int i = 0; i < NX; i++) {
			exp.x0[i] = pkt.x0[i];
			exp.x1[i] = pkt.x1[i];
			exp.s0[i] = pkt.s0[i];
			exp.s1[i] = pkt.s1[i];
		}

		for (int i = 0; i < NU; i++) {
			exp.u0[i] = pkt.u0[i];
			exp.u_nom0[i] = pkt.u_nom0[i];
			exp.u_nom1[i] = pkt.u_nom1[i];
		}

		exp.solver_status = 0;
		exp.timestamp = now;
		_expansion_pub.publish(exp);

		pgf_value_hessian_s hess{};
		hess.seq = pkt.seq;

		for (int i = 0; i < 91; i++) {
			hess.s0_upper[i] = pkt.s0_upper[i];
		}

		hess.timestamp = now;
		_hessian_pub.publish(hess);

		_rx_count++;
		_last_rx = now;
	}
}

void PgfSimBridge::publishHeartbeat()
{
	offboard_control_mode_s msg{};
	msg.direct_actuator = true;
	msg.timestamp = hrt_absolute_time();
	_offboard_mode_pub.publish(msg);
}

void PgfSimBridge::requestOffboardAndArm()
{
	vehicle_command_s cmd{};
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.from_external = true;

	if (!_offboard) {
		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		cmd.param1 = 1.f;   // custom mode
		cmd.param2 = 6.f;   // PX4 offboard
		cmd.timestamp = hrt_absolute_time();
		_vehicle_command_pub.publish(cmd);

	} else if (!_armed) {
		cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
		cmd.param1 = 1.f;
		cmd.timestamp = hrt_absolute_time();
		_vehicle_command_pub.publish(cmd);
	}
}

void PgfSimBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	vehicle_status_s status;

	if (_vehicle_status_sub.update(&status)) {
		_armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_offboard = status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	}

	receivePayloads();
	sendState();

	// Same failsafe shape as quad_px4_offboard: no fresh expansion means the
	// heartbeat stops and PX4 leaves offboard, rather than flying on stale data.
	const bool companion_alive = _last_rx > 0
				     && hrt_elapsed_time(&_last_rx) < _timeout_ms * 1000.f;

	if (companion_alive) {
		publishHeartbeat();

		if (_auto_engage && hrt_elapsed_time(&_started) > kEngageDelay
		    && (!_offboard || !_armed)) {
			requestOffboardAndArm();
		}
	}

	perf_end(_loop_perf);
}

int PgfSimBridge::task_spawn(int argc, char *argv[])
{
	PgfSimBridge *instance = new PgfSimBridge();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		PX4_ERR("init failed");

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int PgfSimBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PgfSimBridge::print_status()
{
	PX4_INFO("tx %u state packets, rx %u payloads", (unsigned)_tx_count, (unsigned)_rx_count);
	PX4_INFO("last payload %.1f ms ago, companion %s",
		 (double)(_last_rx > 0 ? hrt_elapsed_time(&_last_rx) * 1e-3f : -1.f),
		 (_last_rx > 0 && hrt_elapsed_time(&_last_rx) < _timeout_ms * 1000.f) ? "alive" : "quiet");
	PX4_INFO("armed %s, offboard %s", _armed ? "yes" : "no", _offboard ? "yes" : "no");
	perf_print_counter(_loop_perf);
	return 0;
}

int PgfSimBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simulation-only stand-in for the PGF companion computer.

Streams the state estimate to a Python process over UDP and publishes the value
expansion it sends back onto the same uORB topics the uXRCE-DDS client would
have written, so mc_pgf_control can be flown in SITL without ROS 2 or a DDS
agent. Also supplies the offboard direct-actuator heartbeat, withholding it when
the companion goes quiet.

This does not exercise the XRCE-DDS transport; the hardware echo test does that.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pgf_sim_bridge", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int pgf_sim_bridge_main(int argc, char *argv[])
{
	return PgfSimBridge::main(argc, argv);
}
