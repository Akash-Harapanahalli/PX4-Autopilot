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
 * @file MulticopterPgfControl.hpp
 *
 * Runs the Pontryagin gradient flow onboard, driven by a value function
 * expansion a companion computer publishes over DDS at each MPC solve.
 *
 * Why onboard: the flow integrates at gyro rate against the estimator's own
 * output, so the loop carries none of the link latency it would if the
 * companion computer sent motor commands. The law is integral, so it tolerates
 * the noisier full-rate estimate happily.
 *
 * The math lives in src/lib/pgf, which is unit tested against the Python
 * implementation the companion computer runs. This class is the uORB plumbing
 * around it: assemble the state, gate on mode and freshness, convert thrust to
 * a motor command.
 *
 * Engagement follows the existing flight stack: the companion computer streams
 * OffboardControlMode with direct_actuator set, which idles mc_rate_control and
 * control_allocator, and this module then owns actuator_motors. Its heartbeat
 * remains the master kill switch, since commander's offboard-loss failsafe
 * watches that and nothing else.
 */

#pragma once

#include <lib/pgf/PgfFlow.hpp>
#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/pgf_status.h>
#include <uORB/topics/pgf_value_expansion.h>
#include <uORB/topics/pgf_value_hessian.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class MulticopterPgfControl : public ModuleBase<MulticopterPgfControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterPgfControl();
	~MulticopterPgfControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

private:
	void Run() override;

	void updateParameters();

	/** Latch the two halves of the expansion. @return true if one is usable. */
	bool updatePayload();

	/** Assemble the 13-state estimate. @return false if any stream is missing. */
	bool assembleState(pgf::StateVector &x);

	/** True if an estimator reset means the expansion no longer describes this state. */
	bool estimatorReset(const vehicle_local_position_s &pos, const vehicle_attitude_s &att);

	void publishActuators(const pgf::InputVector &u);
	void publishStatus(const pgf::InputVector &u, const pgf::FlowDiagnostics &diag, float frac);

	// Subscriptions. The angular velocity callback is what clocks this module:
	// it fires at the gyro rate, which is the whole point of running here.
	uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _value_expansion_sub{ORB_ID(pgf_value_expansion)};
	uORB::Subscription _value_hessian_sub{ORB_ID(pgf_value_hessian)};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<pgf_status_s> _pgf_status_pub{ORB_ID(pgf_status)};

	pgf::Flow _flow{};

	pgf::Payload _payload{};
	bool _have_expansion{false};
	bool _have_hessian{false};
	uint32_t _expansion_seq{0};
	uint32_t _hessian_seq{0};
	hrt_abstime _payload_arrival{0};   ///< arrival time, not the embedded stamp
	uint64_t _sample_timestamp{0};     ///< when the estimate behind x0 was sampled
	uint64_t _gyro_timestamp_sample{0};

	hrt_abstime _last_run{0};
	hrt_abstime _reset_gate_until{0};
	bool _flow_active{false};

	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _quat_reset_counter{0};
	bool _reset_counters_valid{false};

	// Cached parameters
	float _thrust_a{0.f};
	float _thrust_b{0.f};
	float _thrust_c{0.f};
	int32_t _flow_enabled{1};
	float _timeout_s{0.3f};

	perf_counter_t _loop_perf;
	perf_counter_t _loop_interval_perf;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PGF_FLOW_EN>)  _param_pgf_flow_en,
		(ParamFloat<px4::params::PGF_ALPHA_C>) _param_pgf_alpha_c,
		(ParamFloat<px4::params::PGF_R>)      _param_pgf_r,
		(ParamFloat<px4::params::PGF_WPROX>)  _param_pgf_wprox,
		(ParamFloat<px4::params::PGF_TOUT>)   _param_pgf_tout,
		(ParamFloat<px4::params::PGF_MASS>)   _param_pgf_mass,
		(ParamFloat<px4::params::PGF_IXX>)    _param_pgf_ixx,
		(ParamFloat<px4::params::PGF_IYY>)    _param_pgf_iyy,
		(ParamFloat<px4::params::PGF_IZZ>)    _param_pgf_izz,
		(ParamFloat<px4::params::PGF_KM>)     _param_pgf_km,
		(ParamFloat<px4::params::PGF_M0_X>)   _param_pgf_m0_x,
		(ParamFloat<px4::params::PGF_M0_Y>)   _param_pgf_m0_y,
		(ParamFloat<px4::params::PGF_M0_S>)   _param_pgf_m0_s,
		(ParamFloat<px4::params::PGF_M1_X>)   _param_pgf_m1_x,
		(ParamFloat<px4::params::PGF_M1_Y>)   _param_pgf_m1_y,
		(ParamFloat<px4::params::PGF_M1_S>)   _param_pgf_m1_s,
		(ParamFloat<px4::params::PGF_M2_X>)   _param_pgf_m2_x,
		(ParamFloat<px4::params::PGF_M2_Y>)   _param_pgf_m2_y,
		(ParamFloat<px4::params::PGF_M2_S>)   _param_pgf_m2_s,
		(ParamFloat<px4::params::PGF_M3_X>)   _param_pgf_m3_x,
		(ParamFloat<px4::params::PGF_M3_Y>)   _param_pgf_m3_y,
		(ParamFloat<px4::params::PGF_M3_S>)   _param_pgf_m3_s,
		(ParamFloat<px4::params::PGF_THR_A>)  _param_pgf_thr_a,
		(ParamFloat<px4::params::PGF_THR_B>)  _param_pgf_thr_b,
		(ParamFloat<px4::params::PGF_THR_C>)  _param_pgf_thr_c,
		(ParamFloat<px4::params::PGF_UMAX>)   _param_pgf_umax
	)
};
