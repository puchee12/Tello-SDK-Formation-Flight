import time
import numpy as np
from NatNetClient import NatNetClient


def connectOptitrack(body_id_drone1, body_id_drone2):

	# This will create a new NatNet client
	streamingClient = NatNetClient(body_id_drone1, body_id_drone2)

	# Configure the streaming client to call our rigid body handler on the
	# emulator to send data out.
	streamingClient.newFrameListener = True
	streamingClient.rigidBodyListener = np.zeros((2, 3), dtype=object)

	# Start up the streaming client now that the callbacks are set up.
	# This will run perpetually, and operate on a separate thread.
	streamingClient.run()
	# Slow the OptiTrack threads down?

	# Time to retrieve first state. If no state currently received, the listener
	# just remains the initial value
	start_time = time.time()
	print('Connecting to Opti-Track .....')
	while streamingClient.rigidBodyListener[0, 0] == 0:

		current_time = time.time()
		elapsed_time = current_time - start_time
		if elapsed_time > 10:
			print('Did not receive data from Opti-Track')
			return False

	print('Opti-Track connected')
	return streamingClient



def telloState(streamingClient):

	# Retrieve rigid body data from OptiTrack.
	id_num = streamingClient.rigidBodyListener[:, 0]
	pos = streamingClient.rigidBodyListener[:, 1]
	pos = np.vstack(pos)
	quaternion = streamingClient.rigidBodyListener[:, 2]
	quaternion = np.vstack(quaternion)

	# Rotate coordinates to aircraft standard (forward x, right y, down z) from
	# left x, back y, up z.
	rotated_pos = Cx(np.pi) @ Cz(-np.pi/2) @ pos.T
	rotated_pos = rotated_pos.T * 100		# change to cm

	rotated_quat = Cx(np.pi) @ Cz(-np.pi/2) @ quaternion[:,0:3].T
	rotated_quat = np.concatenate([rotated_quat, np.array([quaternion[:,3]])])
	euler = quaternion2Euler(rotated_quat).T

	return np.array([id_num, rotated_pos, rotated_quat, euler])


def setwaypoint(streamingClient):

	# theta = np.linspace(0, 2*np.pi, 100)
	# radius = 150	# cm
	# x = radius * np.sin(theta)
	# y = radius * np.cos(theta)
	# z = np.ones(100) * -200
	#
	# waypoint = np.zeros([100, 3])
	# waypoint[:, 0] = x
	# waypoint[:, 1] = y
	# waypoint[:, 2] = z

	start_position = telloState(streamingClient)[1][0,:]	# of tello

	# State number of waypoints and list them below. Note units are in cm and 
	# order is (x,y,z) in the standard aircraft coordinate system.
	# Remember, forward x, right y, down z
	num_of_waypoints = 16 # 16 for Flight Path 8 / 8 for Circle
	waypoint = np.zeros([num_of_waypoints, 3])

	# Waypoint 1 is the starting point
	waypoint[0] = start_position

	# Put Waypoint 1 here
	waypoint[1] = waypoint[0] + np.array([14.64, -35.36, 0])

	# Put Waypoint 2 here
	waypoint[2] = waypoint[1] + np.array([35.36, -14.64, 0])

	# Put Waypoint 3 here
	waypoint[3] = waypoint[2] + np.array([35.36, 14.64, 0])

	# Put Waypoint 4 here
	waypoint[4] = waypoint[3] + np.array([14.64, 35.36, 0])

	# Put Waypoint 5 here
	waypoint[5] = waypoint[4] + np.array([-14.64, 35.36, 0])

	# Put Waypoint 6 here
	waypoint[6] = waypoint[5] + np.array([-35.36, 14.64, 0])

	# Put Waypoint 7 here
	waypoint[7] = waypoint[6] + np.array([-35.36, -14.64, 0])

	################### Flight Path 8 Starts Here ###############

	# Put Waypoint 8 here
	waypoint[8] = waypoint[7] + np.array([-14.64, -35.36, 0])

	# Put Waypoint 9 here
	waypoint[9] = waypoint[8] + np.array([-14.64, -35.36, 0])

	# Put Waypoint 10 here
	waypoint[10] = waypoint[9] + np.array([-35.36, -14.64, 0])

	# Put Waypoint 11 here
	waypoint[11] = waypoint[10] + np.array([-35.36, 14.64, 0])

	# Put Waypoint 12 here
	waypoint[12] = waypoint[11] + np.array([-14.64, 35.36, 0])

	# Put Waypoint 13 here
	waypoint[13] = waypoint[12] + np.array([14.64, 35.36, 0])

	# Put Waypoint 14 here
	waypoint[14] = waypoint[13] + np.array([35.36, 14.64, 0])

	# Put Waypoint 15 here
	waypoint[15] = waypoint[14] + np.array([35.36, -14.64, 0])

	###################### 3D Circular ############################

	# # Put Waypoint 1 here
	# waypoint[1] = waypoint[0] + np.array([14.64, -35.36, 10])
	#
	# # Put Waypoint 2 here
	# waypoint[2] = waypoint[1] + np.array([35.36, -14.64, 25])
	#
	# # Put Waypoint 3 here
	# waypoint[3] = waypoint[2] + np.array([35.36, 14.64, 25])
	#
	# # Put Waypoint 4 here
	# waypoint[4] = waypoint[3] + np.array([14.64, 35.36, 10])
	#
	# # Put Waypoint 5 here
	# waypoint[5] = waypoint[4] + np.array([-14.64, 35.36, -10])
	#
	# # Put Waypoint 6 here
	# waypoint[6] = waypoint[5] + np.array([-35.36, 14.64, -25])
	#
	# # Put Waypoint 7 here
	# waypoint[7] = waypoint[6] + np.array([-35.36, -14.64, -25])

	###################### 3D 8 ###################################

	# # Put Waypoint 1 here
	# waypoint[1] = waypoint[0] + np.array([14.64, -35.36, -10])
	#
	# # Put Waypoint 2 here
	# waypoint[2] = waypoint[1] + np.array([35.36, -14.64, -25])
	#
	# # Put Waypoint 3 here
	# waypoint[3] = waypoint[2] + np.array([35.36, 14.64, -25])
	#
	# # Put Waypoint 4 here
	# waypoint[4] = waypoint[3] + np.array([14.64, 35.36, -10])
	#
	# # Put Waypoint 5 here
	# waypoint[5] = waypoint[4] + np.array([-14.64, 35.36, 10])
	#
	# # Put Waypoint 6 here
	# waypoint[6] = waypoint[5] + np.array([-35.36, 14.64, 25])
	#
	# # Put Waypoint 7 here
	# waypoint[7] = waypoint[6] + np.array([-35.36, -14.64, 25])
	#
	# # Put Waypoint 8 here
	# waypoint[8] = waypoint[7] + np.array([-14.64, -35.36, 0])
	#
	# # Put Waypoint 9 here
	# waypoint[9] = waypoint[8] + np.array([-14.64, -35.36, 10])
	#
	# # Put Waypoint 10 here
	# waypoint[10] = waypoint[9] + np.array([-35.36, -14.64, 25])
	#
	# # Put Waypoint 11 here
	# waypoint[11] = waypoint[10] + np.array([-35.36, 14.64, 25])
	#
	# # Put Waypoint 12 here
	# waypoint[12] = waypoint[11] + np.array([-14.64, 35.36, 10])
	#
	# # Put Waypoint 13 here
	# waypoint[13] = waypoint[12] + np.array([14.64, 35.36, -10])
	#
	# # Put Waypoint 14 here
	# waypoint[14] = waypoint[13] + np.array([35.36, 14.64, -25])
	#
	# # Put Waypoint 15 here
	# waypoint[15] = waypoint[14] + np.array([35.36, -14.64, -25])

	# UPDATE NUM OF WAYPOINTS ABOVE
	
	return waypoint


def waypointUpdate(streamingClient, true_state, waypoint):

	# Attain current position (x,y,z)
	current_position = true_state[1]

	try:
		# Relative vector from current position to current waypoint
		r_wd = waypoint[waypointUpdate.current_waypoint] - current_position
	except (IndexError, AttributeError):
		# First time run or all waypoints reached therefore set to start
		waypointUpdate.current_waypoint = 0
		r_wd = waypoint[waypointUpdate.current_waypoint] - current_position

	# Distance to the next waypoint. Transition to next if within 50 cm.
	distance_to_waypoint = np.linalg.norm(r_wd)
	if distance_to_waypoint < 28:
		waypointUpdate.current_waypoint += 1
		# r_wd will  be updated in next iteration to avoid IndexError's.

	return r_wd

def quaternion2Euler(quat):

	# Separate variables for the quaternions
	q0 = quat[3]
	q1 = quat[0]
	q2 = quat[1]
	q3 = quat[2]

	# Calculate the Euler Angles
	theta = np. arctan2(q0 * q2 - q1 * q3,
		np.sqrt((q0 ** 2 + q1 ** 2 - 0.5) ** 2 + (q1 * q2 + q0 * q3) ** 2))
	phi = np. arctan2(q2 * q3 + q0 * q1, q0 ** 2 + q3 ** 2 - 0.5)
	psi = np. arctan2(q1 * q2 + q0 * q3, q0 ** 2 + q1 ** 2 - 0.5)

	# Construct the return array
	euler = np.array([phi, theta, psi])
	return euler


def Cx(angle):

	# Rotation matrix
	rotate_x = np.array([1, 0, 0,
		0, np.cos(angle), np.sin(angle),
		0, -np.sin(angle), np.cos(angle)])

	# Shape correctly to 3 x 3
	rotate_x.shape = (3, 3)

	return rotate_x


def Cy(angle):

	# Rotation matrix
	rotate_y = np.array([np.cos(angle), 0, -np.sin(angle),
			0, 1, 0,
			np.sin(angle), 0, np.cos(angle)])

	# Shape correctly to 3 x 3
	rotate_y.shape = (3, 3)

	return rotate_y


def Cz(angle):

	# Rotation matrix
	rotate_z = np.array([np.cos(angle), np.sin(angle), 0,
		-np.sin(angle), np.cos(angle), 0,
		0, 0, 1])

	# Shape correctly to 3 x 3
	rotate_z.shape = (3, 3)

	return rotate_z