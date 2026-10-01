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

#include "PgfFlow.hpp"

#include <mathlib/mathlib.h>

namespace pgf
{

void Flow::setConfig(const FlowConfig &cfg)
{
	_cfg = cfg;
	_l_uu = 2.f * cfg.r_scalar;
	_q_uu = _l_uu + 2.f * cfg.w_prox;
}

InputVector Flow::clamp(const InputVector &u) const
{
	InputVector out;

	for (int i = 0; i < NU; i++) {
		out(i) = math::constrain(u(i), _cfg.u_min, _cfg.u_max);
	}

	return out;
}

StateVector Flow::alignedState(const StateVector &x, const StateVector &x_hat) const
{
	StateVector out = x;
	float dot = 0.f;

	for (int i = 0; i < 4; i++) {
		dot += x(IDX_Q + i) * x_hat(IDX_Q + i);
	}

	if (dot < 0.f) {
		for (int i = 0; i < 4; i++) {
			out(IDX_Q + i) = -x(IDX_Q + i);
		}
	}

	return out;
}

InputVector Flow::optimalityResidual(const Payload &p, const StateVector &x, float frac) const
{
	const StateVector x_hat = p.x0 + (p.x1 - p.x0) * frac;
	const StateVector s_hat = p.s0 + (p.s1 - p.s0) * frac;
	const InputVector u_nom = p.u_nom0 + (p.u_nom1 - p.u_nom0) * frac;

	const StateVector xa = alignedState(x, x_hat);
	const StateVector w = s_hat + p.S0 * (xa - x_hat);

	return (_u - u_nom) * _l_uu + _model.fu(xa).transpose() * w
	       + (_u - p.u0) * (2.f * _cfg.w_prox);
}

const InputVector &Flow::update(const Payload &p, const StateVector &x, float frac, float dt,
				FlowDiagnostics *diag)
{
	if (!_model.isValid() || !(p.dtau > 0.f) || !(dt > 0.f)) {
		if (diag != nullptr) {
			diag->solve_failed = true;
		}

		return _u;
	}

	const StateVector dx = p.x1 - p.x0;
	const StateVector ds = p.s1 - p.s0;
	const InputVector du_nom = p.u_nom1 - p.u_nom0;

	const StateVector x_hat = p.x0 + dx * frac;
	const StateVector s_hat = p.s0 + ds * frac;
	const InputVector u_nom = p.u_nom0 + du_nom * frac;

	const StateVector xa = alignedState(x, x_hat);
	const StateVector w = s_hat + p.S0 * (xa - x_hat);

	const InputJacobian f_u = _model.fu(xa);
	const matrix::Matrix<float, NU, NX> f_u_t = f_u.transpose();

	// Q_ux = f_u' S0 + <w, f_ux>. The second term is nonzero only in the four
	// quaternion columns, where every rotor's row is the same.
	GainMatrix q_ux = f_u_t * p.S0;
	const matrix::Vector<float, 4> wfux_row = _model.wfuxQuatRow(xa, w);

	for (int i = 0; i < NU; i++) {
		for (int j = 0; j < 4; j++) {
			q_ux(i, IDX_Q + j) += wfux_row(j);
		}
	}

	const InputVector q_u = (_u - u_nom) * _l_uu + f_u_t * w + (_u - p.u0) * (2.f * _cfg.w_prox);

	// A clamped frac means the expansion is frozen because a payload is late,
	// so it no longer drifts with tau.
	InputVector q_utau{};

	if (frac < 1.f) {
		q_utau = (f_u_t * (ds - p.S0 * dx) - du_nom * _l_uu) / p.dtau;
	}

	const float alpha = _cfg.alpha_c / p.dtau;

	// Q_uu is _q_uu times the identity, so its inverse is a scalar division.
	const InputVector u_dot =
		-(q_u * alpha + q_ux * _model.f(xa, _u) + q_utau) / _q_uu;

	// Semi-implicit Euler: (I - dt J) du = dt udot, with
	// J = -Q_uu^-1 (alpha Q_uu + Q_ux f_u) = -(alpha I + Q_ux f_u / _q_uu).
	matrix::SquareMatrix<float, NU> lhs = q_ux * f_u / _q_uu;

	for (int i = 0; i < NU; i++) {
		lhs(i, i) += alpha;
	}

	lhs = matrix::eye<float, NU>() + lhs * dt;

	matrix::SquareMatrix<float, NU> lhs_inv;
	const bool ok = matrix::inv(lhs, lhs_inv);

	if (ok) {
		_u = clamp(_u + lhs_inv * (u_dot * dt));

	} else {
		// Singular within one tick is not something to paper over with an
		// explicit step, which is exactly what this integrator exists to
		// avoid. Hold the last input and let the caller see it.
		if (diag != nullptr) {
			diag->solve_failed = true;
		}
	}

	if (diag != nullptr) {
		diag->qu_norm = q_u.norm();
		diag->tightness = (_q_uu * _cfg.u_max > 1e-6f)
				  ? q_u.norm() / (_q_uu * _cfg.u_max) : 0.f;
		diag->solve_failed = !ok;
	}

	return _u;
}

} // namespace pgf
