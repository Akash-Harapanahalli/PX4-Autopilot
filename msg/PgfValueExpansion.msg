# Pontryagin Gradient Flow: first-order value function expansion from the offboard MPC.
# Published by the companion computer at each MPC solve, consumed by mc_pgf_control.
# Paired with PgfValueHessian by 'seq'; kept under the 512 byte XRCE serial MTU.
#
# State convention (nx = 13): [p_NED(3), v_NED(3), q_wxyz(4) FRD->NED, omega_FRD(3)]
# Input convention (nu = 4): per-rotor thrust in Newtons, index i -> MOTOR{i+1}

uint64 timestamp		# [us] time since system start (microseconds)
uint64 timestamp_sample		# [us] timestamp of the state estimate used as x0

uint32 seq			# solve counter, pairs this message with PgfValueHessian

float32 dtau			# [s] MPC solve interval == first shooting interval

float32[13] x0			# optimal state at shooting node 0
float32[13] x1			# optimal state at shooting node 1
float32[13] s0			# value gradient V_x(x0)
float32[13] s1			# costate at node 1 == V_x(x1)

float32[4] u0			# [N] optimal per-rotor thrust at node 0
float32[4] u_nom0		# [N] stage cost input reference at node 0
float32[4] u_nom1		# [N] stage cost input reference at node 1

uint8 solver_status		# acados solver status, 0 == success
