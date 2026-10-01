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
 * @file PgfFlow.hpp
 *
 * The Pontryagin gradient flow: an integral control law on udot that drives the
 * optimality residual Q_u to zero between MPC solves.
 *
 * The companion computer solves the MPC and ships a local quadratic model of
 * the value function; this runs at gyro rate against the full-rate EKF estimate,
 * so the loop carries no link latency. Ported from
 * pgf-ablation/pgfbench/laws.py:84-107, generalized for a nonzero input
 * reference u_nom because the MPC tracks trajectories, not just hover.
 *
 * Over one interval the expansion is frozen. At interval fraction
 * frac = clamp(t/dtau, 0, 1) and measured state x:
 *
 *   xhat = x0 + frac*(x1 - x0)        shat = s0 + frac*(s1 - s0)
 *   unom = u_nom0 + frac*(u_nom1 - u_nom0)
 *   w    = shat + S0 (x - xhat)
 *
 *   Q_u    = l_uu (u - unom) + f_u' w + 2 w_prox (u - u0)
 *   Q_ux   = f_u' S0 + <w, f_ux>
 *   Q_uu   = (l_uu + 2 w_prox) I                 exactly; the model is affine in u
 *   Q_utau = [f_u' ((s1 - s0) - S0 (x1 - x0)) - l_uu (u_nom1 - u_nom0)] / dtau
 *
 * Imposing Qdot_u = -alpha Q_u along xdot = f(x, u):
 *
 *   udot = -Q_uu^-1 (alpha Q_u + Q_ux f(x, u) + Q_utau)
 *
 * integrated with semi-implicit Euler. That is mandatory, not a refinement: the
 * u-ODE Jacobian is dominated by -Q_uu^-1 f_u' S0 f_u, whose eigenvalues reach
 * ~1e5 on a quadrotor, and explicit Euler chatters bound to bound.
 *
 * l_uu = 2R with NO dtau scaling, which holds only because the OCP uses
 * LINEAR_LS with W = 2*blkdiag(Q, R) and acados' default Riemann cost_scaling.
 * quad_mpc/ocp.py asserts that; if it ever changes, this constant must too.
 */

#pragma once

#include "PgfModel.hpp"

namespace pgf
{

/** The value function expansion for one MPC interval, as received from DDS. */
struct Payload {
	StateVector x0{};       ///< optimal state at shooting node 0
	StateVector x1{};       ///< optimal state at shooting node 1
	StateVector s0{};       ///< value gradient V_x(x0)
	StateVector s1{};       ///< costate at node 1, == V_x(x1)
	matrix::SquareMatrix<float, NX> S0{};   ///< value Hessian V_xx(x0), symmetric
	InputVector u0{};       ///< optimal per-rotor thrust at node 0 [N]
	InputVector u_nom0{};   ///< stage cost input reference at node 0 [N]
	InputVector u_nom1{};   ///< stage cost input reference at node 1 [N]
	float dtau{0.f};        ///< MPC solve interval [s]
};

struct FlowConfig {
	float r_scalar{0.5f};    ///< OCP input weight R; l_uu = 2R
	float alpha_c{20.f};     ///< alpha = alpha_c / dtau
	float w_prox{0.f};       ///< proximal pull toward u0; raise if tightness > 0.4
	float u_min{0.f};        ///< [N] armed idle thrust
	float u_max{0.f};        ///< [N] max thrust per rotor
};

/** Diagnostics for one flow tick, logged to pgf_status for tuning. */
struct FlowDiagnostics {
	float qu_norm{0.f};
	float tightness{0.f};   ///< ||Q_u|| / ((l_uu + 2 w_prox) u_max), target 0.1 .. 0.4
	bool  solve_failed{false};
};

/**
 * Holds the commanded thrust across MPC re-solves. That carry is what makes the
 * law integral, so the input is never reset while flying.
 */
class Flow
{
public:
	void setConfig(const FlowConfig &cfg);
	const FlowConfig &config() const { return _cfg; }

	Model &model() { return _model; }
	const Model &model() const { return _model; }

	/** Set the carried input, e.g. to u0 for a bumpless engage. */
	void reset(const InputVector &u) { _u = clamp(u); }

	const InputVector &input() const { return _u; }

	/**
	 * Advance the carried input by one tick.
	 *
	 * @param p    the frozen expansion for the current interval
	 * @param x    measured state; its quaternion is aligned to the expansion
	 * @param frac interval fraction, clamped to [0, 1] by the caller
	 * @param dt   [s] time since the previous call
	 */
	const InputVector &update(const Payload &p, const StateVector &x, float frac, float dt,
				  FlowDiagnostics *diag = nullptr);

	/** Q_u at the current carried input, without advancing. */
	InputVector optimalityResidual(const Payload &p, const StateVector &x, float frac) const;

private:
	InputVector clamp(const InputVector &u) const;

	/**
	 * Put the measured quaternion in the same hemisphere as the expansion.
	 * S0 (x - xhat) is only meaningful if q and qhat are nearby points in R^4,
	 * and q / -q are the same attitude but antipodal vectors.
	 */
	StateVector alignedState(const StateVector &x, const StateVector &x_hat) const;

	Model _model{};
	FlowConfig _cfg{};
	InputVector _u{};
	float _l_uu{1.f};    ///< 2R
	float _q_uu{1.f};    ///< l_uu + 2 w_prox; Q_uu is this times the identity
};

} // namespace pgf
