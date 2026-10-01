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

#include "MulticopterPgfControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

using namespace pgf;

/// Guard against a stalled work queue producing a nonsense integration step.
static constexpr float kMinDt = 2e-4f;
static constexpr float kMaxDt = 1e-2f;

/// Hold off the flow for this many solve intervals after an estimator reset,
/// so the expansion in hand describes the same state the estimator now reports.
static constexpr float kResetGateIntervals = 3.f;

MulticopterPgfControl::MulticopterPgfControl() :
	ModuleParams(nullptr),
	px4::WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_loop_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
	updateParameters();
}

MulticopterPgfControl::~MulticopterPgfControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool MulticopterPgfControl::init()
{
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void MulticopterPgfControl::updateParameters()
{
	updateParams();

	const float rotor_x[NU] = {_param_pgf_m0_x.get(), _param_pgf_m1_x.get(),
				   _param_pgf_m2_x.get(), _param_pgf_m3_x.get()
				  };
	const float rotor_y[NU] = {_param_pgf_m0_y.get(), _param_pgf_m1_y.get(),
				   _param_pgf_m2_y.get(), _param_pgf_m3_y.get()
				  };
	const float rotor_spin[NU] = {_param_pgf_m0_s.get(), _param_pgf_m1_s.get(),
				      _param_pgf_m2_s.get(), _param_pgf_m3_s.get()
				     };

	_flow.model().setParameters(_param_pgf_mass.get(),
				    matrix::Vector3f(_param_pgf_ixx.get(), _param_pgf_iyy.get(),
						    _param_pgf_izz.get()),
				    rotor_x, rotor_y, rotor_spin, _param_pgf_km.get());

	_thrust_a = _param_pgf_thr_a.get();
	_thrust_b = _param_pgf_thr_b.get();
	_thrust_c = _param_pgf_thr_c.get();
	_flow_enabled = _param_pgf_flow_en.get();
	_timeout_s = _param_pgf_tout.get() * 1e-3f;

	FlowConfig cfg;
	cfg.r_scalar = _param_pgf_r.get();
	cfg.alpha_c = _param_pgf_alpha_c.get();
	cfg.w_prox = _param_pgf_wprox.get();
	cfg.u_min = _thrust_c;                    // an armed rotor always spins
	cfg.u_max = _param_pgf_umax.get();
	_flow.setConfig(cfg);
}

bool MulticopterPgfControl::updatePayload()
{
	pgf_value_expansion_s exp;

	if (_value_expansion_sub.update(&exp)) {
		for (int i = 0; i < NX; i++) {
			_payload.x0(i) = exp.x0[i];
			_payload.x1(i) = exp.x1[i];
			_payload.s0(i) = exp.s0[i];
			_payload.s1(i) = exp.s1[i];
		}

		for (int i = 0; i < NU; i++) {
			_payload.u0(i) = exp.u0[i];
			_payload.u_nom0(i) = exp.u_nom0[i];
			_payload.u_nom1(i) = exp.u_nom1[i];
		}

		_payload.dtau = exp.dtau;
		_expansion_seq = exp.seq;
		_sample_timestamp = exp.timestamp_sample;
		// Arrival time, not the embedded stamp: the uXRCE client already
		// removed its clock offset from timestamp_sample, but the flow's
		// staleness decision should not depend on that offset being good.
		_payload_arrival = hrt_absolute_time();
		_have_expansion = exp.solver_status == 0 && exp.dtau > 0.f;
	}

	pgf_value_hessian_s hess;

	if (_value_hessian_sub.update(&hess)) {
		int k = 0;

		for (int i = 0; i < NX; i++) {
			for (int j = i; j < NX; j++) {
				_payload.S0(i, j) = hess.s0_upper[k];
				_payload.S0(j, i) = hess.s0_upper[k];
				k++;
			}
		}

		_hessian_seq = hess.seq;
		_have_hessian = true;
	}

	return _have_expansion && _have_hessian;
}

bool MulticopterPgfControl::estimatorReset(const vehicle_local_position_s &pos,
		const vehicle_attitude_s &att)
{
	const bool changed = _reset_counters_valid
			     && (pos.xy_reset_counter != _xy_reset_counter
				 || pos.z_reset_counter != _z_reset_counter
				 || pos.vxy_reset_counter != _vxy_reset_counter
				 || pos.vz_reset_counter != _vz_reset_counter
				 || att.quat_reset_counter != _quat_reset_counter);

	_xy_reset_counter = pos.xy_reset_counter;
	_z_reset_counter = pos.z_reset_counter;
	_vxy_reset_counter = pos.vxy_reset_counter;
	_vz_reset_counter = pos.vz_reset_counter;
	_quat_reset_counter = att.quat_reset_counter;
	_reset_counters_valid = true;
	return changed;
}

bool MulticopterPgfControl::assembleState(StateVector &x)
{
	vehicle_local_position_s pos;
	vehicle_attitude_s att;

	if (!_local_position_sub.copy(&pos) || !_attitude_sub.copy(&att)) {
		return false;
	}

	if (!pos.xy_valid || !pos.z_valid || !pos.v_xy_valid || !pos.v_z_valid) {
		return false;
	}

	vehicle_angular_velocity_s rates;

	if (!_angular_velocity_sub.copy(&rates)) {
		return false;
	}

	_gyro_timestamp_sample = rates.timestamp_sample;

	if (estimatorReset(pos, att)) {
		// The expansion in hand was built around the pre-jump state; hold the
		// input until a payload arrives that saw the new one.
		_reset_gate_until = hrt_absolute_time()
				    + static_cast<hrt_abstime>(kResetGateIntervals * _payload.dtau * 1e6f);
	}

	x(IDX_P) = pos.x;
	x(IDX_P + 1) = pos.y;
	x(IDX_P + 2) = pos.z;
	x(IDX_V) = pos.vx;
	x(IDX_V + 1) = pos.vy;
	x(IDX_V + 2) = pos.vz;

	for (int i = 0; i < 4; i++) {
		x(IDX_Q + i) = att.q[i];
	}

	for (int i = 0; i < 3; i++) {
		x(IDX_W + i) = rates.xyz[i];
	}

	normalizeQuaternion(x);
	return true;
}

void MulticopterPgfControl::publishActuators(const InputVector &u)
{
	actuator_motors_s msg{};
	// The gyro sample this cycle ran on, not the MPC's much older state stamp:
	// PX4 uses this for control latency accounting.
	msg.timestamp_sample = _gyro_timestamp_sample;
	msg.reversible_flags = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		msg.control[i] = (i < NU) ? thrustToNormalized(u(i), _thrust_a, _thrust_b, _thrust_c) : NAN;
	}

	msg.timestamp = hrt_absolute_time();
	_actuator_motors_pub.publish(msg);
}

void MulticopterPgfControl::publishStatus(const InputVector &u, const FlowDiagnostics &diag,
		float frac)
{
	pgf_status_s msg{};
	msg.seq = _expansion_seq;

	for (int i = 0; i < NU; i++) {
		msg.u[i] = u(i);
		msg.u_norm[i] = thrustToNormalized(u(i), _thrust_a, _thrust_b, _thrust_c);
	}

	msg.qu_norm = diag.qu_norm;
	msg.tightness = diag.tightness;
	msg.frac = frac;

	const hrt_abstime now = hrt_absolute_time();
	// Total age of the information, i.e. round trip from the estimate the MPC
	// solved on. This is what drives frac: if it exceeds dtau then frac pins at
	// 1, the expansion stops interpolating, and the link latency is the reason.
	msg.payload_age_ms = (_sample_timestamp > 0 && now > _sample_timestamp)
			     ? (now - _sample_timestamp) * 1e-3f : -1.f;

	uint8_t flags = 0;

	if (_flow_active) { flags |= pgf_status_s::FLAG_FLOW_ACTIVE; }

	if (_payload_arrival == 0 || hrt_elapsed_time(&_payload_arrival) > _timeout_s * 1e6f) {
		flags |= pgf_status_s::FLAG_PAYLOAD_STALE;
	}

	if (now < _reset_gate_until) { flags |= pgf_status_s::FLAG_RESET_GATE; }

	if (_flow_enabled == 0) { flags |= pgf_status_s::FLAG_ZOH_MODE; }

	if (_expansion_seq != _hessian_seq) { flags |= pgf_status_s::FLAG_SEQ_MISMATCH; }

	msg.flags = flags;
	msg.timestamp = hrt_absolute_time();
	_pgf_status_pub.publish(msg);
}

void MulticopterPgfControl::Run()
{
	if (should_exit()) {
		_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParameters();
	}

	const bool payload_ok = updatePayload();
	const hrt_abstime now = hrt_absolute_time();
	const bool fresh = payload_ok && _payload_arrival > 0
			   && hrt_elapsed_time(&_payload_arrival) < _timeout_s * 1e6f;

	// Engagement gate. This module may only write actuator_motors while the
	// vehicle is in offboard direct-actuator control, where commander has
	// disabled rate control and allocation. The moment that stops being true,
	// stop publishing so nothing fights over the motors.
	vehicle_control_mode_s control_mode{};
	offboard_control_mode_s offboard_mode{};
	_control_mode_sub.copy(&control_mode);
	_offboard_control_mode_sub.copy(&offboard_mode);

	const bool mode_ok = control_mode.flag_armed
			     && control_mode.flag_control_offboard_enabled
			     && offboard_mode.direct_actuator
			     && !control_mode.flag_control_allocation_enabled;

	StateVector x;
	const bool state_ok = assembleState(x);

	if (!mode_ok || !state_ok || !_flow.model().isValid()) {
		// Track u0 while disengaged so engaging is bumpless.
		if (fresh) {
			_flow.reset(_payload.u0);
		}

		_flow_active = false;
		FlowDiagnostics diag{};
		publishStatus(_flow.input(), diag, 0.f);
		_last_run = now;
		perf_end(_loop_perf);
		return;
	}

	float dt = (_last_run > 0) ? (now - _last_run) * 1e-6f : 0.f;
	_last_run = now;
	dt = math::constrain(dt, kMinDt, kMaxDt);

	FlowDiagnostics diag{};
	float frac = 0.f;

	if (_flow_enabled == 0) {
		// Zero-order hold: replay the MPC's stage-0 input. This is the A/B
		// reference the flow must reproduce exactly when it is switched off.
		if (fresh) {
			_flow.reset(_payload.u0);
		}

	} else if (!fresh || now < _reset_gate_until) {
		// Hold the last input rather than publishing NaN, which would stop the
		// motors outright. The companion computer's heartbeat is what turns a
		// sustained dropout into a failsafe; this only bridges a lost frame.

	} else {
		const float age_s = (now > _sample_timestamp)
				    ? (now - _sample_timestamp) * 1e-6f : 0.f;
		frac = math::constrain(age_s / _payload.dtau, 0.f, 1.f);
		_flow.update(_payload, x, frac, dt, &diag);
	}

	_flow_active = fresh && _flow_enabled != 0 && now >= _reset_gate_until;

	publishActuators(_flow.input());
	publishStatus(_flow.input(), diag, frac);
	perf_end(_loop_perf);
}

int MulticopterPgfControl::task_spawn(int argc, char *argv[])
{
	MulticopterPgfControl *instance = new MulticopterPgfControl();

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

int MulticopterPgfControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPgfControl::print_status()
{
	PX4_INFO("model %s, flow %s", _flow.model().isValid() ? "valid" : "INVALID",
		 _flow_enabled ? "enabled" : "disabled (ZOH)");
	PX4_INFO("expansion seq %u, hessian seq %u, age %.1f ms, active %s",
		 (unsigned)_expansion_seq, (unsigned)_hessian_seq,
		 (double)(_payload_arrival > 0 ? hrt_elapsed_time(&_payload_arrival) * 1e-3f : -1.f),
		 _flow_active ? "yes" : "no");
	const InputVector &u = _flow.input();
	PX4_INFO("u [N] %.3f %.3f %.3f %.3f", (double)u(0), (double)u(1), (double)u(2), (double)u(3));
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int MulticopterPgfControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Pontryagin gradient flow controller.

Consumes a value function expansion published by a companion computer running an
acados MPC (pgf_value_expansion / pgf_value_hessian) and integrates an optimality
flow on the rotor thrusts at gyro rate, publishing actuator_motors directly.

Running the flow here rather than on the companion computer removes the link
latency from the loop and lets it use the estimator's full-rate output.

The module only drives the motors while the vehicle is in offboard mode with
OffboardControlMode.direct_actuator set, which is what idles mc_rate_control and
control_allocator. Set THR_MDL_FAC to 0: this module owns the whole thrust curve
via PGF_THR_A/B/C.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pgf_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mc_pgf_control_main(int argc, char *argv[])
{
	return MulticopterPgfControl::main(argc, argv);
}
