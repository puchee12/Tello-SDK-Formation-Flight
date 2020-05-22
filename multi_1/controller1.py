import numpy as np

def runWaypoint(true_state, streamingClient, r_wd, dt, tello1):
	'''
	Controller for waypoint navigation using OptiTrack (streamingClient) and 
	given vector to next waypoint (r_wd). Projects the waypoint vector into the
	x-y plane and calculates the yaw offset and the forward/back and left/right
	combinations of inputs required to directly track the waypoint.
	'''
	current_orient_euler = true_state[3]

	# Vector projection along x-y plane (height component (z) is zero)
	r_wd_proj = np.array([r_wd[0], r_wd[1], 0])

	# Find the yaw angle (between the vector and the x-axis) through the dot
	# product formula. This gives the yaw angle required to face the waypoint.
	yaw_w = np.arctan2(r_wd_proj[1], r_wd_proj[0])		# in radians

	# Offset angle between drone heading and waypoint heading angle (yaw)
	yaw_w = yaw_w * 180/np.pi   # in degrees
	beta = yaw_w - (current_orient_euler[2] * 180/np.pi)
	# Update frequency is too slow with the Tello's onboard state readings
	# state = tello.readState()
	# beta = yaw_w - state['yaw']		# in degree

	# yaw_w can give values from -180 to 180, the euler angle can range from
	# -180 to 180 hence max beta: 360, min beta = -360.
	# Correct the angle to shortest rotation if exceeding 180 degrees
	if beta > 180:
		beta = beta - 360
	elif beta < -180:
		beta = beta + 360

	# Use the angle to find components of the vector projection in the forward/
	# back direction and the left/right direction.
	signal = np.array([np.linalg.norm(r_wd_proj) * np.sin(beta * np.pi/180),	# Lateral
		np.linalg.norm(r_wd_proj) * np.cos(beta * np.pi/180),	# Longitudinal
		r_wd[2],			# Vertical
		beta])				# yaw

	reference = np.array([0, 0, 0, 0])
	error = signal - reference

	try:
		controllerWaypoint(error, runWaypoint.prev_error, dt, tello1)

	except AttributeError:
		controllerWaypoint(error, error, dt, tello1)		# first run
	runWaypoint.prev_error = error
	return error


def controllerWaypoint(error, prev_error, dt, tello1):

	# Numerical differentiation - first order difference scheme
	error_dot = (error - prev_error) / dt

	# PD constants and controller (Standard form)
	Kp = np.array([0.4, 0.7, 1.0, 1.0])	# lr, fb, ud, yaw
	Td = np.array([0, 0, 0, 0])
	pid_input = Kp * (error + Td * error_dot)

	# Longitudinal to laterial ratio
	ratio = pid_input[1] / pid_input[0]
	if ratio == 0:
		pass

	# Maintain ratio between the limited controller inputs
	pid_input = controllerLimits(pid_input, -100.0, 100.0)
	if abs(ratio) > 1:
		pid_input[0] = (1 / ratio) * pid_input[1]
		# component[1] = limited_component[1]
	else:
		pid_input[1] = ratio * pid_input[0]
		# component[0] = limited_component[0]
	# tello.rc(yaw=int(pid_input[3]))
	tello1.rc(lr=int(pid_input[0]), fb=int(pid_input[1]), ud=-int(pid_input[2]), yaw=int(pid_input[3]))
	# tello1.rc(lr=int(pid_input[0]), fb=int(pid_input[1]), ud=-int(pid_input[2]), yaw=int(0))


def controllerLimits(cont_input, min_limit, max_limit):
	'''
	Tello accepts a maximum of 100 and minimum of -100 for rc inputs hence this
	function prevents higher or lower values. Can also limit to smaller ranges,
	i.e. -50 to 50.

	input: Controller input terms (array)
	min_limit: Minimum controller input cutoff
	max_limit: Maximum controller input cutoff

	returns:
	limited_input: Input but with enforced limits
	'''
	limited_input = np.where(cont_input > max_limit, max_limit, cont_input)
	limited_input = np.where(limited_input < min_limit, min_limit, limited_input)
	return limited_input

