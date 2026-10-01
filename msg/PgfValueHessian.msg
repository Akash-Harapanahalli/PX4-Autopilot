# Pontryagin Gradient Flow: value function Hessian V_xx(x0) from the offboard MPC.
# Split out of PgfValueExpansion so each message fits the 512 byte XRCE serial MTU.
#
# The 13x13 symmetric Hessian is sent as its row-major upper triangle:
#   s0_upper[k] = V_xx[i][j] for i <= j, with k = i*13 - i*(i-1)/2 + (j - i)

uint64 timestamp	# [us] time since system start (microseconds)

uint32 seq		# solve counter, pairs this message with PgfValueExpansion

float32[91] s0_upper	# upper triangle of the symmetrized value Hessian V_xx(x0)
