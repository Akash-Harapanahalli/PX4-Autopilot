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
 * Enable the Pontryagin gradient flow controller
 *
 * Starts mc_pgf_control at boot. The module only drives the motors while the
 * vehicle is in offboard mode with direct actuator control requested.
 *
 * @boolean
 * @reboot_required true
 * @group PGF
 */
PARAM_DEFINE_INT32(PGF_EN, 0);

/**
 * Run the flow, rather than replaying the MPC input
 *
 * Disable to hold the MPC's stage-0 input over each interval (zero order hold).
 * That is the A/B reference the flow must reproduce exactly when switched off,
 * and the right setting for first flights.
 *
 * @boolean
 * @group PGF
 */
PARAM_DEFINE_INT32(PGF_FLOW_EN, 1);

/**
 * Flow gain coefficient
 *
 * The flow drives the optimality residual as Qdot_u = -alpha Q_u, with
 * alpha = PGF_ALPHA_C / dtau. A sweep over the reference implementation found
 * the textbook value of 5 undertunes under a correlated disturbance and that
 * tracking error becomes essentially independent of the solve interval at 20.
 *
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_ALPHA_C, 20.0f);

/**
 * Input weight R of the companion MPC cost
 *
 * MUST equal the R the companion computer built its value expansion with
 * (drone_params.yaml: pgf.r_weight). The flow uses l_uu = 2R; a mismatch
 * silently detunes the loop rather than failing.
 *
 * @min 0.001
 * @decimal 3
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_R, 0.5f);

/**
 * Proximal weight
 *
 * Adds 2*w_prox*(u - u0) to the optimality residual, pulling the flow toward the
 * MPC solution. Only needed on vehicles whose actuator tightness
 * (logged in pgf_status) sits above about 0.4. Leave at 0 otherwise.
 *
 * @min 0.0
 * @decimal 2
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_WPROX, 0.0f);

/**
 * Value expansion timeout
 *
 * Beyond this age the flow holds its last input instead of integrating a stale
 * expansion. It does not stop the motors: the companion computer's offboard
 * heartbeat is what turns a sustained dropout into a failsafe.
 *
 * @unit ms
 * @min 20.0
 * @max 2000.0
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_TOUT, 300.0f);

/**
 * Vehicle mass
 *
 * @unit kg
 * @min 0.0
 * @decimal 3
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_MASS, 0.0f);

/**
 * Moment of inertia about the body x axis
 *
 * @unit kg m^2
 * @min 0.0
 * @decimal 6
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_IXX, 0.0f);

/**
 * Moment of inertia about the body y axis
 *
 * @unit kg m^2
 * @min 0.0
 * @decimal 6
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_IYY, 0.0f);

/**
 * Moment of inertia about the body z axis
 *
 * @unit kg m^2
 * @min 0.0
 * @decimal 6
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_IZZ, 0.0f);

/**
 * Rotor torque to thrust ratio
 *
 * Yaw torque produced per Newton of rotor thrust. Equal to the magnitude of
 * CA_ROTORn_KM; the sign comes from the per-rotor spin parameters.
 *
 * @unit m
 * @min 0.0
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_KM, 0.016f);

/**
 * Rotor 0 x position
 *
 * Rotor hub position in the body frame (FRD), matching CA_ROTOR0_PX.
 *
 * @unit m
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M0_X, 0.0f);

/**
 * Rotor 0 y position
 *
 * @unit m
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M0_Y, 0.0f);

/**
 * Rotor 0 spin direction
 *
 * +1 for counter-clockwise seen from above, -1 for clockwise. Must agree with
 * the sign of CA_ROTOR0_KM.
 *
 * @min -1.0
 * @max 1.0
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M0_S, 1.0f);

/**
 * Rotor 1 x position
 *
 * @unit m
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M1_X, 0.0f);

/**
 * Rotor 1 y position
 *
 * @unit m
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M1_Y, 0.0f);

/**
 * Rotor 1 spin direction
 *
 * +1 for counter-clockwise seen from above, -1 for clockwise.
 *
 * @min -1.0
 * @max 1.0
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M1_S, 1.0f);

/**
 * Rotor 2 x position
 *
 * @unit m
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M2_X, 0.0f);

/**
 * Rotor 2 y position
 *
 * @unit m
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M2_Y, 0.0f);

/**
 * Rotor 2 spin direction
 *
 * +1 for counter-clockwise seen from above, -1 for clockwise.
 *
 * @min -1.0
 * @max 1.0
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M2_S, -1.0f);

/**
 * Rotor 3 x position
 *
 * @unit m
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M3_X, 0.0f);

/**
 * Rotor 3 y position
 *
 * @unit m
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M3_Y, 0.0f);

/**
 * Rotor 3 spin direction
 *
 * +1 for counter-clockwise seen from above, -1 for clockwise.
 *
 * @min -1.0
 * @max 1.0
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_M3_S, -1.0f);

/**
 * Thrust curve quadratic coefficient
 *
 * Per-rotor thrust as T = PGF_THR_A eta^2 + PGF_THR_B eta + PGF_THR_C, with eta
 * the normalized motor command. Fit from bench data by
 * quadrotor-flight-stack/scripts/fit_thrust_map.py. Set THR_MDL_FAC to 0 so PX4
 * does not apply a second curve on top of this one.
 *
 * @unit N
 * @min 0.0
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_THR_A, 0.0f);

/**
 * Thrust curve linear coefficient
 *
 * @unit N
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_THR_B, 0.0f);

/**
 * Thrust curve constant term
 *
 * Thrust an armed rotor produces at zero command, i.e. at ESC idle. Also used as
 * the lower thrust limit, since the vehicle cannot command less.
 *
 * @unit N
 * @min 0.0
 * @decimal 4
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_THR_C, 0.0f);

/**
 * Maximum per-rotor thrust
 *
 * Thrust at full command, i.e. PGF_THR_A + PGF_THR_B + PGF_THR_C. Bounds the
 * flow and normalizes the actuator tightness diagnostic.
 *
 * @unit N
 * @min 0.0
 * @decimal 3
 * @group PGF
 */
PARAM_DEFINE_FLOAT(PGF_UMAX, 0.0f);
