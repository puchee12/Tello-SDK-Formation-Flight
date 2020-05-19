import numpy as np
from tello2 import Tello2

def runWaypoint(true_state, streamingClient, r_wd, dt, tello2):
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
		controllerWaypoint(error, runWaypoint.prev_error, dt, tello2)
		# PID(error, runWaypoint.prev_error, dt, tello2)

	except AttributeError:
		controllerWaypoint(error, error, dt, tello2)		# first run
		# PID(error, error, dt, tello2)		# first run
	runWaypoint.prev_error = error
	return error


def controllerWaypoint(error, prev_error, dt, tello2):

	# Numerical differentiation - first order difference scheme
	error_dot = (error - prev_error) / dt

	# PD constants and controller (Standard form)
	Kp = np.array([0.7, 0.8, 0.8, 1.0])	# lr, fb, ud, yaw
	Td = np.array([0, 0, 0, 0])
	pid_input = Kp * (error + Td * error_dot)

	# print(pid_input)
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
	tello2.rc(lr=int(pid_input[0]), fb=int(pid_input[1]), ud=-int(pid_input[2]), yaw=int(pid_input[3]))
	# tello2.rc(lr=int(pid_input[0]), fb=int(pid_input[1]), ud=-int(pid_input[2]), yaw=int(0))



def PID(error, prev_error, dt, tello2):
	''' PID controller sends commands to tello based on error. '''
	# Integral needs to be persistent hence is an attribute of PID
	try:
		PID.integral += error * dt
		#print(PID.integral)
	except AttributeError:
		PID.integral = np.zeros(len(error))

	try:
		PID.error_dot1 = (error - prev_error) / dt
		PID.error_dot2 = PID.error_dot2
		PID.error_dot3 = PID.error_dot3
		PID.error_dot4 = PID.error_dot4
	except AttributeError:
		PID.error_dot1 = (error - prev_error) / dt
		PID.error_dot2 = np.zeros(len(error))
		PID.error_dot3 = np.zeros(len(error))
		PID.error_dot4 = np.zeros(len(error))

	# Numerical differentiation - first order difference scheme
	# error_dot = (error - prev_error) / dt
	error_dot = (PID.error_dot1 + PID.error_dot2 + PID.error_dot3 + PID.error_dot4)/4
	
	# Log data
	# config.data_log['PID_edot_lr (17)'] = int(error_dot[0])
	# config.data_log['PID_edot_ud (19)'] = int(error_dot[1])
	# config.data_log['PID_edot_yaw (20)'] = int(error_dot[2])
	# config.data_log['PID_edot_fb (18)'] = int(error_dot[3])

	# PID constants and controller (Standard form)
	# Gains for left/right, up/down, yaw, and forward/back (fb not used)
	Kp = np.array([3.3, 6.0, 2.0, 1.2])
	Ti = 100
	Td = np.array([0.9, 0.2, 0.0, 0.0])
	pid_input = -Kp * (error + 0*PID.integral / Ti + Td * error_dot)

	# Log data
	# config.data_log['PID_cmd_lr (21)'] = int(pid_input[0])
	# config.data_log['PID_cmd_ud (23)'] = int(pid_input[1])
	# config.data_log['PID_cmd_yaw (24)'] = -int(pid_input[2])
	# config.data_log['PID_cmd_fb (22)'] = int(pid_input[3])

	# Limit PID input to max of 100, and minimum of -100
	pid_input = controllerLimits(pid_input, -100, 100)

	# Controller inputs to tello
	tello2.rc(lr=int(pid_input[0]), ud=int(pid_input[1]), yaw=-int(pid_input[2]),
				fb=10)#fb=-int(0.25*pid_input[3]))



def PID2(error, prev_error, dt, tello2):
	''' This controller was written to use the pose estimation approach from
	runPID but it never seemed to work better than the original controller.
	Requires changes in yaw border as well from RunPID(). '''
	# Numerical differentiation - first order difference scheme
	error_dot = (error - prev_error) / dt

	# PD constants and controller (Standard form)
	Kp = np.array([3.3, 6.0, 2.0, 1])	# lr, ud, yaw, fb
	Td = np.array([0.3, 0.0, 0.0, 0.0])
	pid_input = Kp * (error + Td * error_dot)

	# Longitudinal to laterial ratio
	ratio = pid_input[3] / pid_input[0]

	# Maintain ratio between the limited controller inputs
	pid_input = controllerLimits(pid_input, -100.0, 100.0)
	if abs(ratio) > 1:
		pid_input[0] = (1 / ratio) * pid_input[3]
	else:
		pid_input[3] = ratio * pid_input[0]

	tello.rc(lr=int(pid_input[0]), ud=-int(pid_input[1]), yaw=-int(pid_input[2]),
		fb=int(pid_input[3]))

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

